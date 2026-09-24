"""The immutable inputs of one routing session.

Milestone 5 Slice 2a: every value the session constructor used to derive from
its keyword arguments and from `RoutingConfig` is a field of one frozen
`SessionSettings`, built by `SessionSettings.from_arguments(...)`. The
validation and normalisation rules in `from_arguments` are the constructor's
rules moved unchanged (same order, same messages, same defaults); per-run state
the phases write stays on the session.

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

from translation.crossing_modes import is_lidar_mode, normalize_crossing_mode
from translation.route_order import normalize_net_order
from translation.route_rust_crossing_plan import _effective_crossing_search_loss
from translation.route_rust_types import RipupRerouteConfig

DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING = 2


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
    enable_jps4: bool
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
    node_depths: dict[str, int] | None
    node_ranks: dict[str, int] | None
    edge_ranks: dict[str, dict[str, int]] | None

    # ---- crossings ------------------------------------------------------------
    # Consumed by the crossing plan, the dispatch and the final verification.
    enable_crossings: bool
    crossing_loss: float
    crossing_mode: str
    crossing_half_size_cells: int
    min_straight_cells_per_crossing: int
    allow_only_expected_crossings: bool
    effective_allow_only_expected_crossings: bool
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
    def from_arguments(
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
        crossing_mode: str = "lidar-pure",
        crossing_half_size_cells: int = 0,
        min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
        foreign_port_keepout_cells: int = 0,
        fanout_access_mode: str | None = None,
        allow_only_expected_crossings: bool = True,
        crossing_guidance_net_names: frozenset[str] | None = None,
        defer_realization: bool = False,
        enable_checked_endpoint_correction: bool = True,
        config: RoutingConfig | None = None,
    ) -> "SessionSettings":
        """Validate and normalise the session keywords into one frozen record.

        The rules are the ones the session constructor applied before Milestone 5
        Slice 2a, in the same order and with the same messages.
        """
        resolved_config: RoutingConfig = (
            config if config is not None else RoutingConfig.from_environment()
        )
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
            crossing_loss=float(crossing_loss),
            config=resolved_config.crossing_plan,
        )
        if crossing_half_size_cells < 0:
            raise ValueError("crossing_half_size_cells must be non-negative")
        if min_straight_cells_per_crossing < 0:
            raise ValueError("min_straight_cells_per_crossing must be non-negative")
        if foreign_port_keepout_cells < 0:
            raise ValueError("foreign_port_keepout_cells must be non-negative")
        raw_fanout_access_mode = (
            resolved_config.fanout.fanout_access_mode
            if resolved_config.fanout.fanout_access_mode is not None
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

        return SessionSettings(
            config=resolved_config,
            router_config=resolved_config.router,
            unrouted_layout=unrouted_layout,
            schematic=schematic,
            obstacle_config=obstacle_config,
            include_heater_obstacles=include_heater_obstacles,
            debug_dir=debug_dir,
            debug_prefix=debug_prefix,
            debug_route_indices=debug_route_indices,
            debug_stop_after_route_index=debug_stop_after_route_index,
            debug_timing=debug_timing,
            verbose_route_diagnostics=verbose_route_diagnostics,
            collect_route_stats=collect_route_stats,
            collect_attempt_diagnostics=collect_attempt_diagnostics,
            collect_pipeline_timing=(
                debug_timing or collect_route_stats or collect_attempt_diagnostics
            ),
            allow_45_degree_turns=allow_45_degree_turns,
            enable_jps4=enable_jps4,
            use_indexed_heap=use_indexed_heap,
            enable_simple_routes=enable_simple_routes,
            primitive_ordering=primitive_ordering,
            heuristic_mode=heuristic_mode,
            heap_tie_breaker=heap_tie_breaker,
            proactive_congestion_weight=proactive_congestion_weight,
            proactive_congestion_radius_cells=proactive_congestion_radius_cells,
            max_iterations=max_iterations,
            routing_window_scale=routing_window_scale,
            net_order=normalize_net_order(net_order),
            # Optional depth map (instance -> hops from a true source) computed
            # on the benchmark's original netlist; contribution 2 passes it so
            # that the tile stubs of the derived netlist do not scramble the
            # depth layers of the surrounding bands (2026-09-16, mm128 fan-in).
            net_order_depth_by_node=(
                dict(net_order_depth_by_node) if net_order_depth_by_node else None
            ),
            node_depths=node_depths,
            node_ranks=node_ranks,
            edge_ranks=edge_ranks,
            enable_crossings=enable_crossings,
            crossing_loss=crossing_loss,
            crossing_mode=crossing_mode,
            crossing_half_size_cells=crossing_half_size_cells,
            min_straight_cells_per_crossing=min_straight_cells_per_crossing,
            allow_only_expected_crossings=allow_only_expected_crossings,
            effective_allow_only_expected_crossings=effective_allow_only_expected_crossings,
            crossing_search_loss=crossing_search_loss,
            crossing_guidance_net_names=(
                frozenset(crossing_guidance_net_names)
                if crossing_guidance_net_names is not None
                else None
            ),
            foreign_port_keepout_cells=foreign_port_keepout_cells,
            fanout_access_mode_normalized=fanout_access_mode_normalized,
            ripup_reroute_config=ripup_reroute_config,
            enable_internal_photonic_probe_verification=(
                enable_internal_photonic_probe_verification
            ),
            route_width_um=route_width_um,
            route_layer=route_layer,
            bend_radius_um=bend_radius_um,
            defer_realization=defer_realization,
            enable_checked_endpoint_correction=enable_checked_endpoint_correction,
        )
