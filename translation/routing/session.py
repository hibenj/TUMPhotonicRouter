"""The routing session shell: construction, the nine-phase `run`, the pipeline
timers and the module-level `route_nets_rust` entry point.

Milestone 5 Slice 1 moved every phase body and its stage-private helpers into
the sibling stage modules of this package, as module-level functions taking the
session as their first argument. The class keeps a one-line forwarder per moved
method, so `session.<helper>(...)` inside a stage body, the unbound lookups in
the tests and the fake sessions built with `object.__new__` all keep working.
The forwarders go away in Slice 2, when the stages take typed inputs and
outputs and `run` passes one stage's output to the next."""

from __future__ import annotations

import importlib
import math
import time

from pathlib import Path
from typing import Any

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.config import RouterConfig, RoutingConfig

from translation.crossing_modes import is_lidar_mode, normalize_crossing_mode
from translation.route_order import normalize_net_order
from translation.route_rust_crossing_plan import _effective_crossing_search_loss
from translation.route_rust_types import RipupRerouteConfig, RustRouteDebugArtifacts

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

DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING = 2


class _RouteNetsRustSession:
    # `_FanoutAnchor` lives in `route_jobs` since Milestone 5 Slice 1; the
    # attribute keeps `session._FanoutAnchor(...)` working.
    # compatibility forwarder, removed in Milestone 5 Slice 2
    _FanoutAnchor = route_jobs_stage._FanoutAnchor

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
        self.config: RoutingConfig = config if config is not None else RoutingConfig.from_environment()
        if route_width_um <= 0:
            raise ValueError("route_width_um must be > 0")
        if max_iterations <= 0:
            raise ValueError("max_iterations must be > 0")
        if not math.isfinite(float(proactive_congestion_weight)) or proactive_congestion_weight < 0:
            raise ValueError("proactive_congestion_weight must be finite and non-negative")
        if proactive_congestion_radius_cells < 0:
            raise ValueError("proactive_congestion_radius_cells must be non-negative")
        if crossing_loss < 0:
            raise ValueError("crossing_loss must be non-negative")
        crossing_mode = normalize_crossing_mode(crossing_mode)
        effective_allow_only_expected_crossings = bool(allow_only_expected_crossings)
        if is_lidar_mode(crossing_mode):
            # lidar-pure and lidar-guided: crossings are never a whitelist
            effective_allow_only_expected_crossings = False
        crossing_search_loss = _effective_crossing_search_loss(
            enable_crossings=bool(enable_crossings),
            crossing_mode=crossing_mode,
            crossing_loss=float(crossing_loss),
            config=self.config.crossing_plan,
        )
        if crossing_half_size_cells < 0:
            raise ValueError("crossing_half_size_cells must be non-negative")
        if min_straight_cells_per_crossing < 0:
            raise ValueError("min_straight_cells_per_crossing must be non-negative")
        if foreign_port_keepout_cells < 0:
            raise ValueError("foreign_port_keepout_cells must be non-negative")
        raw_fanout_access_mode = (
            self.config.fanout.fanout_access_mode
            if self.config.fanout.fanout_access_mode is not None
            else ("legacy-runway" if fanout_access_mode is None else str(fanout_access_mode))
        )
        fanout_access_mode_normalized = raw_fanout_access_mode.strip().lower().replace("_", "-")
        fanout_mode_aliases = {
            "": "legacy-runway",
            "legacy": "legacy-runway",
            "legacy-runway": "legacy-runway",
            "staggered": "legacy-runway",
            "staggered-runway": "legacy-runway",
            "runway": "legacy-runway",
            "0": "off",
            "false": "off",
            "none": "off",
            "disabled": "off",
            "disable": "off",
            "off": "off",
            "anchor": "static-stubs",
            "anchors": "static-stubs",
            "anchor-pre-spread": "static-stubs",
            "pre-spread": "static-stubs",
            "spread-stubs": "static-stubs",
            "static-stub": "static-stubs",
            "static-stubs": "static-stubs",
            "virtual-ports": "static-stubs",
        }
        fanout_access_mode_normalized = fanout_mode_aliases.get(
            fanout_access_mode_normalized,
            fanout_access_mode_normalized,
        )
        if fanout_access_mode_normalized not in {"legacy-runway", "off", "static-stubs"}:
            raise ValueError(
                "fanout_access_mode must be one of 'legacy-runway', 'off', or 'static-stubs'"
            )
        if debug_stop_after_route_index is not None and debug_stop_after_route_index < 1:
            raise ValueError("debug_stop_after_route_index must be >= 1")

        self.unrouted_layout = unrouted_layout
        self.schematic = schematic
        self.obstacle_config = obstacle_config
        self.debug_dir = debug_dir
        self.debug_prefix = debug_prefix
        self.debug_route_indices = debug_route_indices
        self.debug_stop_after_route_index = debug_stop_after_route_index
        self.route_width_um = route_width_um
        self.route_layer = route_layer
        self.allow_45_degree_turns = allow_45_degree_turns
        self.bend_radius_um = bend_radius_um
        self.enable_jps4 = enable_jps4
        self.use_indexed_heap = use_indexed_heap
        self.enable_simple_routes = enable_simple_routes
        self.primitive_ordering = primitive_ordering
        self.heuristic_mode = heuristic_mode
        self.net_order = normalize_net_order(net_order)
        # Optional depth map (instance -> hops from a true source) computed
        # on the benchmark's original netlist; contribution 2 passes it so
        # that the tile stubs of the derived netlist do not scramble the
        # depth layers of the surrounding bands (2026-09-16, mm128 fan-in).
        self.net_order_depth_by_node: dict[str, int] | None = (
            dict(net_order_depth_by_node) if net_order_depth_by_node else None
        )
        self.heap_tie_breaker = heap_tie_breaker
        self.proactive_congestion_weight = proactive_congestion_weight
        self.proactive_congestion_radius_cells = proactive_congestion_radius_cells
        self.max_iterations = max_iterations
        self.routing_window_scale = routing_window_scale
        self.debug_timing = debug_timing
        self.verbose_route_diagnostics = verbose_route_diagnostics
        self.collect_route_stats = collect_route_stats
        self.collect_attempt_diagnostics = collect_attempt_diagnostics
        self.enable_internal_photonic_probe_verification = (
            enable_internal_photonic_probe_verification
        )
        self.include_heater_obstacles = include_heater_obstacles
        self.ripup_reroute_config = ripup_reroute_config
        self.enable_crossings = enable_crossings
        self.node_depths = node_depths
        self.node_ranks = node_ranks
        self.edge_ranks = edge_ranks
        self.crossing_loss = crossing_loss
        self.crossing_mode = crossing_mode
        self.crossing_half_size_cells = crossing_half_size_cells
        self.min_straight_cells_per_crossing = min_straight_cells_per_crossing
        self.foreign_port_keepout_cells = foreign_port_keepout_cells
        self.allow_only_expected_crossings = allow_only_expected_crossings
        self.crossing_guidance_net_names = (
            frozenset(crossing_guidance_net_names)
            if crossing_guidance_net_names is not None
            else None
        )
        self.defer_realization = defer_realization
        self.enable_checked_endpoint_correction = enable_checked_endpoint_correction
        self.effective_allow_only_expected_crossings = effective_allow_only_expected_crossings
        self.crossing_search_loss = crossing_search_loss
        self.fanout_access_mode_normalized = fanout_access_mode_normalized
        self.router_config: RouterConfig = self.config.router

        self.rust_backend = _load_rust_backend()
        if self.rust_backend is None:
            raise RuntimeError(
                "Rust router backend is not available. Build it with `cargo build` "
                "or `maturin develop` so photonic_router._rust can be imported."
            )

        self.routed_layout = self.unrouted_layout.copy()
        self.routed_layout.name = "routed_layout_rust"

        self.collect_pipeline_timing = (
            self.debug_timing or self.collect_route_stats or self.collect_attempt_diagnostics
        )
        self.route_nets_timings_s: dict[str, float] = {}

    def run(self) -> tuple[Component, RustRouteDebugArtifacts]:
        obstacle_map, crossing_device_info, obstacle_svg = obstacle_context.build_static_obstacle_context(self)
        nets = self.schematic.netlist.routes
        router_setup.configure_router_and_grid(self, obstacle_map)
        route_jobs, endpoint_port_specs_by_instance, dense_port_runway_length_by_spec = (
            route_jobs_stage.build_route_jobs_and_fanout_clustering(self, nets)
        )
        if self.config.search.long_straight_exempt_dense_fanout is True:
            # 2026-09-17 (multiportmmi_128x128): the fan-in / fan-out bands of
            # dense multi-port instances route without the long-straight
            # penalty, so their lanes may pack in parallel; every other net
            # keeps the configured weight. See PyPhotonicRouter's
            # `long_straight_exempt_net_ids`.
            exempt_ids = sorted(
                set(self.fanout_anchor_source_net_ids) | set(self.fanout_anchor_target_net_ids)
            )
            if hasattr(self.router, "set_long_straight_exempt_net_ids"):
                self.router.set_long_straight_exempt_net_ids(exempt_ids)
                print(f"      - long-straight penalty exempt for {len(exempt_ids)} dense fan-out net(s)")
        route_jobs, foreign_port_keepout_cells_by_instance = (
            crossing_plan_stage.build_crossing_plan_and_port_footprints(
                self,
                route_jobs,
                endpoint_port_specs_by_instance,
                dense_port_runway_length_by_spec,
                obstacle_map,
                crossing_device_info,
            )
        )
        route_jobs, t_astar_start = handoff.finalize_route_jobs_and_static_handoff(
            self,
            route_jobs,
            foreign_port_keepout_cells_by_instance,
            obstacle_map,
        )

        dispatch.dispatch_native_routing(self, route_jobs)

        routed_net_records, astar_elapsed_s = finalize.finalize_routing_results(
            self,
            route_jobs, t_astar_start
        )

        routed_net_records, illegal_realized_crossings = verify_repair.repair_and_verify_final_geometry(
            self,
            routed_net_records
        )

        debug_artifacts = realize.realize_and_assemble_debug_artifacts(
            self,
            route_jobs,
            routed_net_records,
            illegal_realized_crossings,
            obstacle_map,
            obstacle_svg,
            astar_elapsed_s,
        )
        return self.routed_layout, debug_artifacts

    def _pipeline_timer_start(self) -> float:
        return time.perf_counter() if self.collect_pipeline_timing else 0.0

    def _record_pipeline_timing(self, name: str, start_s: float) -> None:
        if self.collect_pipeline_timing:
            self.route_nets_timings_s[name] = self.route_nets_timings_s.get(name, 0.0) + (
                time.perf_counter() - start_s
            )

    def _record_elapsed(self, bucket_name: str, start_s: float, *, failed: bool = False) -> None:
        if not self.collect_timing:
            return
        self.route_timing_buckets[bucket_name].record_elapsed(
            time.perf_counter() - start_s,
            failed=failed,
        )

    # ---- obstacle_context forwarders ------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _build_static_obstacle_context(self, *args: Any, **kwargs: Any):
        return obstacle_context.build_static_obstacle_context(self, *args, **kwargs)

    # ---- router_setup forwarders ----------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _configure_router_and_grid(self, *args: Any, **kwargs: Any):
        return router_setup.configure_router_and_grid(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    @staticmethod
    def _fanout_int_or_default(*args: Any, **kwargs: Any):
        return router_setup._fanout_int_or_default(*args, **kwargs)

    # ---- route_jobs forwarders ------------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _angle_to_step(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._angle_to_step(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _angle_to_unit_vector(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._angle_to_unit_vector(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _append_arc_from_tangencies(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._append_arc_from_tangencies(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _append_circular_stub_bend(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._append_circular_stub_bend(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _append_grid_step(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._append_grid_step(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _append_realized_stub_bend(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._append_realized_stub_bend(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _append_stub_point(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._append_stub_point(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _build_route_jobs_and_fanout_clustering(self, *args: Any, **kwargs: Any):
        return route_jobs_stage.build_route_jobs_and_fanout_clustering(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _build_static_fanout_anchors(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._build_static_fanout_anchors(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _build_static_fanout_target_anchors(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._build_static_fanout_target_anchors(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _centerline_grid_cells(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._centerline_grid_cells(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _cross2(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._cross2(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _dense_fanout_group_size(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._dense_fanout_group_size(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _dense_fanout_min_ports(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._dense_fanout_min_ports(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _dense_fanout_min_ports_for(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._dense_fanout_min_ports_for(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _dense_source_port_runway_lengths(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._dense_source_port_runway_lengths(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _dense_target_port_runway_lengths(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._dense_target_port_runway_lengths(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _equalize_dense_runway_reach(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._equalize_dense_runway_reach(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _fanout_stub_bend_steps(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._fanout_stub_bend_steps(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _fanout_stub_centerline_um(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._fanout_stub_centerline_um(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _grid_cell_center_um(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._grid_cell_center_um(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _in_bounds(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._in_bounds(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _inflated_cells(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._inflated_cells(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _is_dense_source_fanout_group(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._is_dense_source_fanout_group(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _is_dense_source_fanout_instance(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._is_dense_source_fanout_instance(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _is_dense_target_fanout_group(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._is_dense_target_fanout_group(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _is_dense_target_fanout_instance(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._is_dense_target_fanout_instance(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _orientation_to_angle(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._orientation_to_angle(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _port_access_rule_for(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._port_access_rule_for(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _rotate_left_vector(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._rotate_left_vector(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _rotate_right_vector(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._rotate_right_vector(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _straight_static_stub_centerline_um(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._straight_static_stub_centerline_um(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _two_bend_static_stub_centerline_um(self, *args: Any, **kwargs: Any):
        return route_jobs_stage._two_bend_static_stub_centerline_um(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def port_to_grid_state(self, *args: Any, **kwargs: Any):
        return route_jobs_stage.port_to_grid_state(self, *args, **kwargs)

    # ---- crossing_plan_stage forwarders ---------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _build_crossing_plan_and_port_footprints(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage.build_crossing_plan_and_port_footprints(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _cell_in_raw_static(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._cell_in_raw_static(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _cells_in_raw_static_geometry(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._cells_in_raw_static_geometry(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _endpoint_state_for_lane_assignment(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._endpoint_state_for_lane_assignment(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _heater_opening_rect_ranges_by_y(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._heater_opening_rect_ranges_by_y(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _instance_ref_by_name(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._instance_ref_by_name(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _instance_static_geometry_open_cells(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._instance_static_geometry_open_cells(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _raw_static_cells_by_y(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._raw_static_cells_by_y(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _raw_static_rect_ranges_by_y(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._raw_static_rect_ranges_by_y(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _rect_ranges_by_y(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._rect_ranges_by_y(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _resolve_port_footprint_cells(self, *args: Any, **kwargs: Any):
        return crossing_plan_stage._resolve_port_footprint_cells(self, *args, **kwargs)

    # ---- handoff forwarders ---------------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _debug_hoist_instance_first(self, *args: Any, **kwargs: Any):
        return handoff._debug_hoist_instance_first(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _endpoint_bump_candidate_open_cells_for_state(self, *args: Any, **kwargs: Any):
        return handoff._endpoint_bump_candidate_open_cells_for_state(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _filter_dense_port_opening(self, *args: Any, **kwargs: Any):
        return handoff._filter_dense_port_opening(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _finalize_route_jobs_and_static_handoff(self, *args: Any, **kwargs: Any):
        return handoff.finalize_route_jobs_and_static_handoff(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _foreign_keepout_open_cells_for_spec(self, *args: Any, **kwargs: Any):
        return handoff._foreign_keepout_open_cells_for_spec(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _opened_cells_for_spec(self, *args: Any, **kwargs: Any):
        return handoff._opened_cells_for_spec(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _route_job_grid_span(self, *args: Any, **kwargs: Any):
        return handoff._route_job_grid_span(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _snap_nearly_collinear_states(self, *args: Any, **kwargs: Any):
        return handoff._snap_nearly_collinear_states(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _snap_same_heading_minimum_bend_offset(self, *args: Any, **kwargs: Any):
        return handoff._snap_same_heading_minimum_bend_offset(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _states_and_openings(self, *args: Any, **kwargs: Any):
        return handoff._states_and_openings(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _topological_net_route_order(self, *args: Any, **kwargs: Any):
        return handoff._topological_net_route_order(self, *args, **kwargs)

    # ---- dispatch forwarders --------------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _append_centerline_points(self, *args: Any, **kwargs: Any):
        return dispatch._append_centerline_points(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _cells_in_rect(self, *args: Any, **kwargs: Any):
        return dispatch._cells_in_rect(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _clearance_exempt_cell_set_for_job(self, *args: Any, **kwargs: Any):
        return dispatch._clearance_exempt_cell_set_for_job(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _clearance_exempt_cells_for_job(self, *args: Any, **kwargs: Any):
        return dispatch._clearance_exempt_cells_for_job(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _committed_dynamic_cells(self, *args: Any, **kwargs: Any):
        return dispatch._committed_dynamic_cells(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _committed_dynamic_cells_for_attempt(self, *args: Any, **kwargs: Any):
        return dispatch._committed_dynamic_cells_for_attempt(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _corridor_clearance_diagnostic(self, *args: Any, **kwargs: Any):
        return dispatch._corridor_clearance_diagnostic(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _direction_reaches_target_ray(self, *args: Any, **kwargs: Any):
        return dispatch._direction_reaches_target_ray(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _dispatch_native_routing(self, *args: Any, **kwargs: Any):
        return dispatch.dispatch_native_routing(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _export_route_svg(self, *args: Any, **kwargs: Any):
        return dispatch._export_route_svg(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _fanout_stubbed_centerline(self, *args: Any, **kwargs: Any):
        return dispatch._fanout_stubbed_centerline(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _finalize_committed_route(self, *args: Any, **kwargs: Any):
        return dispatch._finalize_committed_route(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _foreign_keepout_cleanup_cells_for_job(self, *args: Any, **kwargs: Any):
        return dispatch._foreign_keepout_cleanup_cells_for_job(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _foreign_keepout_cleanup_cells_for_spec(self, *args: Any, **kwargs: Any):
        return dispatch._foreign_keepout_cleanup_cells_for_spec(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _record_native_batch_timings(self, *args: Any, **kwargs: Any):
        return dispatch._record_native_batch_timings(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _record_route(self, *args: Any, **kwargs: Any):
        return dispatch._record_route(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _report_long_straight_congestion(self, *args: Any, **kwargs: Any):
        return dispatch._report_long_straight_congestion(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _route_attempt_diagnostics(self, *args: Any, **kwargs: Any):
        return dispatch._route_attempt_diagnostics(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _route_cells_from_router(self, *args: Any, **kwargs: Any):
        return dispatch._route_cells_from_router(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _route_engine_summary(self, *args: Any, **kwargs: Any):
        return dispatch._route_engine_summary(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _routing_endpoint_center_um(self, *args: Any, **kwargs: Any):
        return dispatch._routing_endpoint_center_um(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _source_lower_bounds(self, *args: Any, **kwargs: Any):
        return dispatch._source_lower_bounds(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _state_openings_for_job(self, *args: Any, **kwargs: Any):
        return dispatch._state_openings_for_job(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _static_cells_in_rect(self, *args: Any, **kwargs: Any):
        return dispatch._static_cells_in_rect(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _timing_start(self, *args: Any, **kwargs: Any):
        return dispatch._timing_start(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _write_failed_log(self, *args: Any, **kwargs: Any):
        return dispatch._write_failed_log(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _write_route_diagnostics(self, *args: Any, **kwargs: Any):
        return dispatch._write_route_diagnostics(self, *args, **kwargs)

    # ---- finalize forwarders --------------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _append_target_fanout_stubs_after_correction(self, *args: Any, **kwargs: Any):
        return finalize._append_target_fanout_stubs_after_correction(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _apply_all_endpoint_corrections_for_net_ids(self, *args: Any, **kwargs: Any):
        return finalize._apply_all_endpoint_corrections_for_net_ids(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _apply_checked_endpoint_corrections_for_net_ids(self, *args: Any, **kwargs: Any):
        return finalize._apply_checked_endpoint_corrections_for_net_ids(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
        self, *args: Any, **kwargs: Any
    ):
        return finalize._apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
            self, *args, **kwargs
        )

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _apply_crossing_aware_endpoint_corrections_for_net_ids(self, *args: Any, **kwargs: Any):
        return finalize._apply_crossing_aware_endpoint_corrections_for_net_ids(
            self, *args, **kwargs
        )

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
        self, *args: Any, **kwargs: Any
    ):
        return finalize._apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
            self, *args, **kwargs
        )

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _classify_net_for_endpoint_correction(self, *args: Any, **kwargs: Any):
        return finalize._classify_net_for_endpoint_correction(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _current_crossing_points_by_net_id(self, *args: Any, **kwargs: Any):
        return finalize._current_crossing_points_by_net_id(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _endpoint_correction_crossing_net_ids(self, *args: Any, **kwargs: Any):
        return finalize._endpoint_correction_crossing_net_ids(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _finalize_routing_results(self, *args: Any, **kwargs: Any):
        return finalize.finalize_routing_results(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _record_terminal_bump_distance_check_candidates(self, *args: Any, **kwargs: Any):
        return finalize._record_terminal_bump_distance_check_candidates(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _route_target_angle(self, *args: Any, **kwargs: Any):
        return finalize._route_target_angle(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _route_target_grid_center_um(self, *args: Any, **kwargs: Any):
        return finalize._route_target_grid_center_um(self, *args, **kwargs)

    # ---- verify_repair forwarders ---------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _add_keepout_rect(self, *args: Any, **kwargs: Any):
        return verify_repair._add_keepout_rect(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _add_keepout_square(self, *args: Any, **kwargs: Any):
        return verify_repair._add_keepout_square(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _add_segment_keepout_cells(self, *args: Any, **kwargs: Any):
        return verify_repair._add_segment_keepout_cells(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _cells_from_um_bbox(self, *args: Any, **kwargs: Any):
        return verify_repair._cells_from_um_bbox(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _final_crossing_repair_batches(self, *args: Any, **kwargs: Any):
        return verify_repair._final_crossing_repair_batches(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _final_crossing_repair_keepout_cells(self, *args: Any, **kwargs: Any):
        return verify_repair._final_crossing_repair_keepout_cells(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _final_crossing_repair_net_ids(self, *args: Any, **kwargs: Any):
        return verify_repair._final_crossing_repair_net_ids(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _grid_cell_from_raw_point(self, *args: Any, **kwargs: Any):
        return verify_repair._grid_cell_from_raw_point(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _grid_rect_from_grid_bbox_text(self, *args: Any, **kwargs: Any):
        return verify_repair._grid_rect_from_grid_bbox_text(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _grid_rect_from_um_bbox(self, *args: Any, **kwargs: Any):
        return verify_repair._grid_rect_from_um_bbox(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _illegal_crossing_grid_cell(self, *args: Any, **kwargs: Any):
        return verify_repair._illegal_crossing_grid_cell(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _illegal_crossing_keepout_radius(self, *args: Any, **kwargs: Any):
        return verify_repair._illegal_crossing_keepout_radius(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _make_photonic_verification_probe_layout(self, *args: Any, **kwargs: Any):
        return verify_repair._make_photonic_verification_probe_layout(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _net_id_by_name(self, *args: Any, **kwargs: Any):
        return verify_repair._net_id_by_name(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _photonic_issue_keepout_cells(self, *args: Any, **kwargs: Any):
        return verify_repair._photonic_issue_keepout_cells(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _photonic_issue_net_ids(self, *args: Any, **kwargs: Any):
        return verify_repair._photonic_issue_net_ids(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _photonic_repair_failure_preview(self, *args: Any, **kwargs: Any):
        return verify_repair._photonic_repair_failure_preview(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _polygon_bbox_um(self, *args: Any, **kwargs: Any):
        return verify_repair._polygon_bbox_um(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _refresh_photonic_verification(self, *args: Any, **kwargs: Any):
        return verify_repair._refresh_photonic_verification(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _refresh_realized_crossing_verification(self, *args: Any, **kwargs: Any):
        return verify_repair._refresh_realized_crossing_verification(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _repair_and_verify_final_geometry(self, *args: Any, **kwargs: Any):
        return verify_repair.repair_and_verify_final_geometry(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _repair_final_illegal_crossings(self, *args: Any, **kwargs: Any):
        return verify_repair._repair_final_illegal_crossings(self, *args, **kwargs)

    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _repair_final_photonic_issues(self, *args: Any, **kwargs: Any):
        return verify_repair._repair_final_photonic_issues(self, *args, **kwargs)

    # ---- realize forwarders ---------------------------------------------------
    # compatibility forwarder, removed in Milestone 5 Slice 2
    def _realize_and_assemble_debug_artifacts(self, *args: Any, **kwargs: Any):
        return realize.realize_and_assemble_debug_artifacts(self, *args, **kwargs)


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
