"""The immutable inputs of one routing session.

Milestone 5 Slice 2a: every value the session constructor used to derive from
its keyword arguments and from `RoutingConfig` is a field of one frozen
`SessionSettings`, built by `SessionSettings.from_options(...)` from the
caller's `SessionOptions` (`options.py`). The validation and normalisation
rules are the constructor's rules moved unchanged (same order, same messages);
per-run state the phases write stays on the session.

The fields are grouped by their consumer: configuration, layout and debug,
search, crossings, fan-out and port access, repair, realization.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.config import RouterConfig, RoutingConfig
from photonic_router.crossings.crossing_modes import normalize_crossing_mode
from photonic_router.crossings.route_rust_crossing_plan import effective_crossing_search_loss
from photonic_router.placement_analysis import PlacementAnalysis
from photonic_router.records.route_order import normalize_net_order
from photonic_router.records.route_rust_types import RipupRerouteConfig
from photonic_router.routing.options import SessionOptions


@dataclass(frozen=True)
class SessionSettings:
    """The routing session's inputs, fixed for the whole run."""

    # ---- configuration --------------------------------------------------------
    # The typed configuration tree (Milestone 1) and its Rust-side sub-tree.
    config: RoutingConfig
    router_config: RouterConfig

    # ---- layout and debug -----------------------------------------------------
    # Consumed by the obstacle context, the debug SVG registry and the printers.
    unrouted_layout: Component
    schematic: Schematic
    obstacle_config: object | None
    include_heater_obstacles: bool
    debug_dir: str | Path | None
    debug_prefix: str
    debug_route_indices: set[int] | None
    debug_stop_after_route_index: int | None
    debug_timing: bool
    verbose_route_diagnostics: bool
    collect_route_stats: bool
    collect_attempt_diagnostics: bool
    collect_pipeline_timing: bool

    # ---- search ---------------------------------------------------------------
    # Consumed by the router setup (A* config) and the kernel dispatch.
    allow_45_degree_turns: bool
    use_indexed_heap: bool
    enable_simple_routes: bool
    primitive_ordering: str
    heuristic_mode: str
    heap_tie_breaker: str
    proactive_congestion_weight: float
    proactive_congestion_radius_cells: int
    max_iterations: int
    routing_window_scale: float | None
    net_order: str
    net_order_depth_by_node: dict[str, int] | None
    #: the flow's analysis of this design (`placement_analysis.py`); None when
    #: the session was entered without one and analyses the design itself
    placement_analysis: PlacementAnalysis | None
    #: > 0 runs the detour pass (phase 7b, `detour_pass.py`) with this factor
    detour_pass_factor: float

    # ---- crossings ------------------------------------------------------------
    # Consumed by the crossing plan, the dispatch and the final verification.
    enable_crossings: bool
    crossing_loss: float
    crossing_mode: str
    crossing_half_size_cells: int
    crossing_search_loss: float
    crossing_guidance_net_names: frozenset[str] | None

    # ---- fan-out and port access ----------------------------------------------
    # Consumed by the route-job builder and the static handoff.
    foreign_port_keepout_cells: int
    fanout_access_mode_normalized: str

    # ---- repair ---------------------------------------------------------------
    # Consumed by the static handoff (repair config) and the final repair passes.
    ripup_reroute_config: RipupRerouteConfig | None
    enable_internal_photonic_probe_verification: bool

    # ---- realization ----------------------------------------------------------
    # Consumed by the router setup (cell sizes), the finalizer and the realizer.
    route_width_um: float
    route_layer: tuple[int, int]
    bend_radius_um: float | None
    defer_realization: bool
    enable_checked_endpoint_correction: bool

    @staticmethod
    def from_options(
        unrouted_layout: Component,
        schematic: Schematic,
        options: SessionOptions,
    ) -> SessionSettings:
        """Validate the options of a session and resolve them into its settings.

        Raises `ValueError` for a value outside its range. The configuration
        is read from the environment when the options carry none.
        """
        search = options.search
        crossings = options.crossings
        port_access = options.port_access
        debug = options.debug
        realization = options.realization
        resolved_config: RoutingConfig = (
            options.config if options.config is not None else RoutingConfig.from_environment()
        )
        if realization.route_width_um <= 0:
            raise ValueError("route_width_um must be > 0")
        if search.max_iterations <= 0:
            raise ValueError("max_iterations must be > 0")
        if (
            not math.isfinite(float(search.proactive_congestion_weight))
            or search.proactive_congestion_weight < 0
        ):
            raise ValueError("proactive_congestion_weight must be finite and non-negative")
        if search.proactive_congestion_radius_cells < 0:
            raise ValueError("proactive_congestion_radius_cells must be non-negative")
        if crossings.crossing_loss < 0:
            raise ValueError("crossing_loss must be non-negative")
        crossing_mode = normalize_crossing_mode(crossings.crossing_mode)
        crossing_search_loss = effective_crossing_search_loss(
            enable_crossings=bool(crossings.enable_crossings),
            crossing_loss=float(crossings.crossing_loss),
            config=resolved_config.crossing_plan,
        )
        if crossings.crossing_half_size_cells < 0:
            raise ValueError("crossing_half_size_cells must be non-negative")
        if port_access.foreign_port_keepout_cells < 0:
            raise ValueError("foreign_port_keepout_cells must be non-negative")
        raw_fanout_access_mode = (
            resolved_config.fanout.fanout_access_mode
            if resolved_config.fanout.fanout_access_mode is not None
            else (
                "legacy-runway"
                if port_access.fanout_access_mode is None
                else str(port_access.fanout_access_mode)
            )
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
        if (
            debug.debug_stop_after_route_index is not None
            and debug.debug_stop_after_route_index < 1
        ):
            raise ValueError("debug_stop_after_route_index must be >= 1")

        return SessionSettings(
            config=resolved_config,
            router_config=resolved_config.router,
            unrouted_layout=unrouted_layout,
            schematic=schematic,
            obstacle_config=options.obstacle_config,
            include_heater_obstacles=options.include_heater_obstacles,
            debug_dir=debug.debug_dir,
            debug_prefix=debug.debug_prefix,
            debug_route_indices=debug.debug_route_indices,
            debug_stop_after_route_index=debug.debug_stop_after_route_index,
            debug_timing=debug.debug_timing,
            verbose_route_diagnostics=debug.verbose_route_diagnostics,
            collect_route_stats=debug.collect_route_stats,
            collect_attempt_diagnostics=debug.collect_attempt_diagnostics,
            collect_pipeline_timing=(
                debug.debug_timing or debug.collect_route_stats or debug.collect_attempt_diagnostics
            ),
            allow_45_degree_turns=search.allow_45_degree_turns,
            use_indexed_heap=search.use_indexed_heap,
            enable_simple_routes=search.enable_simple_routes,
            primitive_ordering=search.primitive_ordering,
            heuristic_mode=search.heuristic_mode,
            heap_tie_breaker=search.heap_tie_breaker,
            proactive_congestion_weight=search.proactive_congestion_weight,
            proactive_congestion_radius_cells=search.proactive_congestion_radius_cells,
            max_iterations=search.max_iterations,
            routing_window_scale=search.routing_window_scale,
            net_order=normalize_net_order(search.net_order),
            # Optional depth map (instance -> hops from a true source) computed
            # on the benchmark's original netlist; contribution 2 passes it so
            # that the tile stubs of the derived netlist do not scramble the
            # depth layers of the surrounding bands (2026-09-16, mm128 fan-in).
            net_order_depth_by_node=(
                dict(search.net_order_depth_by_node) if search.net_order_depth_by_node else None
            ),
            placement_analysis=crossings.placement_analysis,
            detour_pass_factor=float(search.detour_pass_factor),
            enable_crossings=crossings.enable_crossings,
            crossing_loss=crossings.crossing_loss,
            crossing_mode=crossing_mode,
            crossing_half_size_cells=crossings.crossing_half_size_cells,
            crossing_search_loss=crossing_search_loss,
            crossing_guidance_net_names=(
                frozenset(crossings.crossing_guidance_net_names)
                if crossings.crossing_guidance_net_names is not None
                else None
            ),
            foreign_port_keepout_cells=port_access.foreign_port_keepout_cells,
            fanout_access_mode_normalized=fanout_access_mode_normalized,
            ripup_reroute_config=options.ripup_reroute_config,
            enable_internal_photonic_probe_verification=(
                options.enable_internal_photonic_probe_verification
            ),
            route_width_um=realization.route_width_um,
            route_layer=realization.route_layer,
            bend_radius_um=realization.bend_radius_um,
            defer_realization=realization.defer_realization,
            enable_checked_endpoint_correction=realization.enable_checked_endpoint_correction,
        )
