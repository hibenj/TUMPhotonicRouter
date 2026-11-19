"""The options of one routing session, grouped by what reads them.

`SessionOptions` is the one argument that carries a caller's choices into
`route_nets_rust` and `route_match_and_realize`; the flow builds it from its
`FlowOptions` (`flow_optical.py`). Every record is frozen and every field
has the default the session has always had, so `SessionOptions()` is the
default session. `SessionSettings.from_options` (`settings.py`) validates
the options and resolves them into the record the routing phases read.

`MatchingOptions` and `CircuitMetadata` are the two further inputs of
`route_match_and_realize`: what path-length matching should do, and what
the benchmark knows about its circuit beyond the netlist.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from photonic_router.config import RoutingConfig
from photonic_router.placement_analysis import PlacementAnalysis
from photonic_router.records.route_rust_types import (
    DEFAULT_MEANDER_MAX_HEIGHT_UM,
    RipupRerouteConfig,
)


@dataclass(frozen=True)
class SearchOptions:
    """The single-net search and the order in which nets are routed."""

    #: False omits the +-45-degree turn primitives
    allow_45_degree_turns: bool = True
    #: benchmark-only queue experiment; measured slower than the default heap
    use_indexed_heap: bool = False
    #: False skips the straight/L/Z candidates and sends every net to A*
    enable_simple_routes: bool = True
    #: benchmark-only primitive iteration order: "library",
    #: "long_straight_first" or "target_biased"
    primitive_ordering: str = "library"
    #: "distance" or "heading_aware"
    heuristic_mode: str = "heading_aware"
    heap_tie_breaker: str = "smaller_g"
    #: soft cost per blocked side-neighbour cell beside a straight move
    proactive_congestion_weight: float = 0.0
    #: sideways radius, in cells, in which those neighbours are counted
    proactive_congestion_radius_cells: int = 0
    #: state expansions allowed per route attempt
    max_iterations: int = 500_000
    routing_window_scale: float | None = None
    #: net order within the topological layers (`records/route_order.py`)
    net_order: str = "topological"
    #: instance depths of the original netlist for the net order; contribution
    #: 2 passes them because the tile stubs of its derived netlist would
    #: scramble the layers
    net_order_depth_by_node: dict[str, int] | None = None
    #: > 0 runs the detour pass after routing with this factor
    detour_pass_factor: float = 0.0


@dataclass(frozen=True)
class CrossingOptions:
    """Whether routes may cross and what the search knows about crossings."""

    enable_crossings: bool = False
    crossing_loss: float = 0.0
    #: "lidar-pure" (the baseline) or "lidar-guided" (contribution 1)
    crossing_mode: str = "lidar-pure"
    #: keep-out half size of a crossing in cells; 0 derives it from the cell
    crossing_half_size_cells: int = 0
    #: contribution 2's router fallback: the nets the guided search may plan
    #: crossings between; None means every net
    crossing_guidance_net_names: frozenset[str] | None = None
    #: the flow's analysis of the design for the guided search; None for the
    #: baseline, and for a guided session that analyses the design itself
    placement_analysis: PlacementAnalysis | None = None


@dataclass(frozen=True)
class PortAccessOptions:
    """How routes leave and reach the ports of dense instances."""

    #: extra keep-out, in cells, in front of every port that only the port's
    #: own net (and same-instance fan-out) may open
    foreign_port_keepout_cells: int = 0
    #: "legacy-runway" (staggered runway reservations, the default when None),
    #: "off", or "static-stubs" (pre-routed fan-out stubs to virtual anchors)
    fanout_access_mode: str | None = None


@dataclass(frozen=True)
class DebugOptions:
    """Debug artifacts, printing and the collection of statistics."""

    #: directory for the debug SVGs; None writes none
    debug_dir: str | Path | None = None
    debug_prefix: str = "route"
    #: 1-based net indices that get a per-route SVG; None means every net
    debug_route_indices: set[int] | None = None
    #: 1-based index after which routing stops; keep-outs and the crossing
    #: context are still built from the whole netlist
    debug_stop_after_route_index: int | None = None
    debug_timing: bool = False
    #: per-net progress and the search's timing buckets; failures always print
    verbose_route_diagnostics: bool = False
    collect_route_stats: bool = False
    collect_attempt_diagnostics: bool = False


@dataclass(frozen=True)
class RealizationOptions:
    """The geometry of a realized route and when it is realized."""

    route_width_um: float = 0.5
    route_layer: tuple[int, int] = (1, 0)
    #: minimum bend radius, rounded up to whole cells; None uses the default
    bend_radius_um: float | None = None
    #: True keeps the routes and skips the polygons, for transforms that run
    #: before realization (path-length matching)
    defer_realization: bool = False
    #: False skips the checked grid-to-port correction pass
    enable_checked_endpoint_correction: bool = True


@dataclass(frozen=True)
class SessionOptions:
    """Everything a caller chooses for one routing session."""

    search: SearchOptions = field(default_factory=SearchOptions)
    crossings: CrossingOptions = field(default_factory=CrossingOptions)
    port_access: PortAccessOptions = field(default_factory=PortAccessOptions)
    debug: DebugOptions = field(default_factory=DebugOptions)
    realization: RealizationOptions = field(default_factory=RealizationOptions)
    #: the static obstacle map's configuration; None uses the builder's default
    obstacle_config: object | None = None
    include_heater_obstacles: bool = False
    ripup_reroute_config: RipupRerouteConfig | None = None
    #: True runs the expensive probe of the realized layout inside the
    #: session; the flow verifies the final layout itself and leaves it off
    enable_internal_photonic_probe_verification: bool = False
    #: the typed configuration tree; None reads `RoutingConfig.from_environment()`
    config: RoutingConfig | None = None


@dataclass(frozen=True)
class MatchingOptions:
    """What path-length matching does after the routes are found."""

    enable_path_length_matching: bool = False
    #: also align the outputs with each other
    path_length_match_outputs: bool = False
    #: node names of groups the design leaves unmatched on purpose; a
    #: residual there is reported, a residual anywhere else fails the run
    path_length_declared_unmatched_groups: tuple[str, ...] = ()
    #: "all" (every output to the latest) or "depth" (per depth)
    path_length_output_scope: str = "all"
    path_length_meander_height_um: float = DEFAULT_MEANDER_MAX_HEIGHT_UM
    #: True makes the meander planner build its profile (timers, counters,
    #: per-candidate attempts); the plans do not depend on it
    debug_meanders: bool = False


@dataclass(frozen=True)
class CircuitMetadata:
    """What a benchmark declares about its circuit beyond the netlist.

    Read by the path-length analysis; every field may be None.
    """

    node_types: dict[str, str] | None = None
    internal_delays_um: dict[str, float] | None = None
    input_ports_by_component: dict[str, tuple[str, ...]] | None = None
