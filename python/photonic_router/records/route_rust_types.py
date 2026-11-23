"""Shared data structures for the Rust-backed photonic router."""

from __future__ import annotations

import enum
import math
from collections.abc import Iterable as IterableABC
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import SupportsIndex, SupportsInt

from gdsfactory.component import Component
from gdsfactory.typings import Port

from photonic_router.records.route_keys import PortRef, RoutedEdgeKey

PRIMITIVE_TRANSITION_CLASSES = (
    "straight_short",
    "straight_long",
    "bend45",
    "bend90",
)

DEFAULT_BEND_RADIUS_UM = 10.0
DEFAULT_MEANDER_MAX_HEIGHT_UM = 80.0


@dataclass(frozen=True)
class OpticalRouteClearancePolicy:
    """Grid radii used for optical routing and PLM occupancy.

    Static obstacles are already rasterized with their own physical clearance.
    Dynamic routes need two radii: one to expand existing dynamic obstacles
    during centerline search, and one to reserve the routed waveguide keepout
    for later routes. PLM uses the committed keepout as occupied space, while
    its candidate box clearance stays explicit so it can be changed separately.
    """

    route_width_um: float
    grid_size_um: float
    route_clearance_um: float
    waveguide_core_radius_cells: int
    dynamic_obstacle_search_expansion_radius_cells: int
    dynamic_route_commit_keepout_radius_cells: int
    dynamic_route_core_radius_cells: int
    plm_registered_route_keepout_radius_cells: int
    plm_candidate_box_clearance_radius_cells: int

    @classmethod
    def from_dimensions(
        cls,
        *,
        route_width_um: float,
        grid_size_um: float,
        route_clearance_um: float,
        plm_candidate_box_clearance_radius_cells: int = 0,
    ) -> OpticalRouteClearancePolicy:
        """Derive the cell radii from the physical dimensions, all in um.

        The core radius is half the route width in cells, rounded up; the
        keep-out radius is half the width plus the clearance in cells,
        rounded up, and never below the core radius. Search expansion uses
        the core radius, the commit keep-out and the PLM registered keep-out
        the keep-out radius; ``dynamic_route_core_radius_cells`` is always 0.
        A clearance of zero or less is taken as zero and makes the core and
        keep-out radii 0 as well. ``plm_candidate_box_clearance_radius_cells``
        is passed through, a negative value as 0.

        Raises ``ValueError`` when the route width or the grid size is not
        finite and positive, or the clearance is infinite or not a number.
        """
        route_width_um = float(route_width_um)
        grid_size_um = float(grid_size_um)
        route_clearance_um = float(route_clearance_um)
        if math.isnan(route_clearance_um):
            # `max(0.0, nan)` is 0.0: the clamp below would hide it
            raise ValueError("route_clearance_um must be finite")
        route_clearance_um = max(0.0, route_clearance_um)
        if not math.isfinite(route_width_um) or route_width_um <= 0.0:
            raise ValueError("route_width_um must be finite and > 0")
        if not math.isfinite(grid_size_um) or grid_size_um <= 0.0:
            raise ValueError("grid_size_um must be finite and > 0")
        if not math.isfinite(route_clearance_um):
            raise ValueError("route_clearance_um must be finite")

        if route_clearance_um <= 0.0:
            core_radius_cells = 0
            keepout_radius_cells = 0
        else:
            core_radius_cells = max(
                0,
                math.ceil((route_width_um / 2.0) / grid_size_um),
            )
            keepout_radius_cells = max(
                core_radius_cells,
                math.ceil(((route_width_um / 2.0) + route_clearance_um) / grid_size_um),
            )
        box_clearance_radius_cells = max(
            0,
            int(plm_candidate_box_clearance_radius_cells),
        )
        return cls(
            route_width_um=route_width_um,
            grid_size_um=grid_size_um,
            route_clearance_um=route_clearance_um,
            waveguide_core_radius_cells=core_radius_cells,
            dynamic_obstacle_search_expansion_radius_cells=core_radius_cells,
            dynamic_route_commit_keepout_radius_cells=keepout_radius_cells,
            dynamic_route_core_radius_cells=0,
            plm_registered_route_keepout_radius_cells=keepout_radius_cells,
            plm_candidate_box_clearance_radius_cells=box_clearance_radius_cells,
        )

    def to_debug_dict(self) -> dict[str, float | int]:
        """The policy as a flat dict keyed by field name, for debug and analysis records."""
        return {
            "route_width_um": self.route_width_um,
            "grid_size_um": self.grid_size_um,
            "route_clearance_um": self.route_clearance_um,
            "waveguide_core_radius_cells": self.waveguide_core_radius_cells,
            "dynamic_obstacle_search_expansion_radius_cells": (
                self.dynamic_obstacle_search_expansion_radius_cells
            ),
            "dynamic_route_commit_keepout_radius_cells": (
                self.dynamic_route_commit_keepout_radius_cells
            ),
            "dynamic_route_core_radius_cells": self.dynamic_route_core_radius_cells,
            "plm_registered_route_keepout_radius_cells": (
                self.plm_registered_route_keepout_radius_cells
            ),
            "plm_candidate_box_clearance_radius_cells": (
                self.plm_candidate_box_clearance_radius_cells
            ),
        }


def bend_radius_cells_from_um(
    bend_radius_um: float | None,
    *,
    grid_size_um: float,
) -> int:
    """Convert a minimum bend radius in micrometers to grid cells."""
    if bend_radius_um is None:
        bend_radius_um = DEFAULT_BEND_RADIUS_UM
    bend_radius_um = float(bend_radius_um)
    grid_size_um = float(grid_size_um)
    if not math.isfinite(bend_radius_um) or bend_radius_um <= 0.0:
        raise ValueError("bend_radius_um must be finite and > 0")
    if not math.isfinite(grid_size_um) or grid_size_um <= 0.0:
        raise ValueError("grid_size_um must be finite and > 0")
    return max(1, math.ceil(bend_radius_um / grid_size_um))


def _empty_primitive_counter_dict() -> dict[str, int]:
    return dict.fromkeys(PRIMITIVE_TRANSITION_CLASSES, 0)


def _get_route_counter_dict(route_obj: object | None, attr: str) -> dict[str, int]:
    counters = _empty_primitive_counter_dict()
    if route_obj is None:
        return counters
    values = getattr(route_obj, attr, None)
    if not isinstance(values, IterableABC):
        return counters
    sequence = list(values)
    for name, value in zip(PRIMITIVE_TRANSITION_CLASSES, sequence, strict=False):
        try:
            counters[name] = int(value)
        except (TypeError, ValueError):
            counters[name] = 0
    return counters


def _add_counter_dict(target: dict[str, int], source: Mapping[str, int]) -> None:
    for name in PRIMITIVE_TRANSITION_CLASSES:
        target[name] = int(target.get(name, 0)) + int(source.get(name, 0))


def as_float(value: object, default: float = 0.0) -> float:
    """Convert a number or a numeric string to float; anything else gives `default`.

    Used on values read from kernel objects and dictionaries whose type is not
    guaranteed. A non-finite float is returned as it is.
    """
    if not isinstance(value, (int, float, str, bytes, bytearray)):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value: object, default: int = 0) -> int:
    """Convert a number or a numeric string to int; anything else gives `default`.

    A float is truncated towards zero.
    """
    if not isinstance(value, (int, float, str, bytes, bytearray)):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def int_of(value: object) -> int:
    """``int(value)`` for a value whose type is not known.

    Accepts what ``int`` accepts and raises what it raises: ``TypeError`` for
    a value that is not a number or a string, ``ValueError`` for a string
    that is not a number. For the callers that skip such a value.
    """
    if not isinstance(value, (int, str, bytes, bytearray, SupportsInt, SupportsIndex)):
        raise TypeError(f"cannot convert {type(value).__name__} to int")
    return int(value)


def int_or_none(value: object) -> int | None:
    """``int(value)``, or None for a value that ``int`` does not convert.

    For records that carry a net id or an index as an untyped value: the
    caller skips the entry that has none.
    """
    try:
        return int_of(value)
    except (TypeError, ValueError):
        return None


def _get_route_int_stat(route_obj: object, attr: str) -> int:
    return max(0, as_int(getattr(route_obj, attr, 0), 0))


@dataclass(frozen=True)
class RustRouteDebugArtifacts:
    """What a routing session returns besides the layout.

    The routed-net records, the search statistics and the grid and obstacle
    data that path-length matching, verification and reporting read
    afterwards. ``obstacle_svg`` and ``route_svgs`` are the debug SVG files
    of the run. ``routed_edge_lengths_um`` is the recorded length per routed
    edge. ``static_blocked_cells`` are the obstacle map's blocked cells,
    sorted, without the port-access reservations.
    ``realization_grid_spec`` is (width in cells, height in cells, cell size
    in um, origin x in um, origin y in um); ``routable_bbox_um`` is (x min,
    y min, x max, y max). ``route_nets_timings_s`` maps a timing bucket's
    name to seconds.
    """

    obstacle_svg: Path | None
    route_svgs: list[Path]
    routed_edge_lengths_um: dict[RoutedEdgeKey, float]
    routed_net_records: list[RoutedNetRecord] = field(default_factory=list)
    route_search_summary: RouteSearchSummary = field(default_factory=lambda: RouteSearchSummary())
    route_attempt_records: list[RouteAttemptRecord] = field(default_factory=list)
    static_blocked_cells: tuple[tuple[int, int], ...] = ()
    static_obstacle_count: int = 0
    static_port_open_count: int = 0
    #: nets the detour pass (phase 7b) re-routed; None when it did not run
    detour_pass_reroutes: int | None = None
    port_alignment_diagnostics: list[dict[str, object]] = field(default_factory=list)
    route_nets_timings_s: dict[str, float] = field(default_factory=dict)
    realization_grid_spec: tuple[int, int, float, float, float] | None = None
    realization_allow_45_degree_turns: bool = True
    realization_bend_radius_cells: int = 4
    crossing_plan_info: dict[str, object] | None = None
    # Chip-boundary keepout: the die routes may use (`StaticObstacleMapData.routable_bbox`).
    routable_bbox_um: tuple[float, float, float, float] | None = None


@dataclass(frozen=True)
class RoutedNetRecord:
    """One routed net as the session records it.

    ``source`` and ``target`` are the net's two ports, ``route_obj`` the
    kernel's route. ``total_length_um`` is the net's length as currently
    recorded. Once an endpoint correction has changed it,
    ``base_total_length_um`` keeps the length from before the first
    correction (None otherwise) and ``corrected_centerline_um`` holds the
    corrected centerline (points in um; empty otherwise). ``net_id`` is the
    id of the net's ``RouteJob``.
    ``opened_cells`` are the grid cells that were opened for the net's
    endpoints. The port centers are in um, the port orientations in degrees.
    ``endpoint_correction_error`` is the message of a failed endpoint
    correction; the photonic verification reports it as an issue.
    """

    net_name: str
    source: PortRef
    target: PortRef
    route_obj: object
    total_length_um: float
    net_id: int | None = None
    meander_auto_plan: dict[str, object] | None = None
    opened_cells: tuple[tuple[int, int], ...] = ()
    source_port_center_um: tuple[float, float] | None = None
    target_port_center_um: tuple[float, float] | None = None
    source_port_orientation_deg: float | None = None
    target_port_orientation_deg: float | None = None
    base_total_length_um: float | None = None
    corrected_centerline_um: tuple[tuple[float, float], ...] = ()
    endpoint_correction_error: str | None = None


@dataclass(frozen=True)
class RouteJob:
    """One net to route: a link of the schematic's netlist with its two ports resolved.

    ``inst1``/``port1`` name the source and ``inst2``/``port2`` the target;
    ``source_port`` and ``target_port`` are those ports on the layout being
    routed. ``net_id`` numbers the links from 1 in the netlist's order and
    is the identity the obstacle map and the bookkeeping key on; it never
    changes. ``route_index`` starts out equal to it and becomes the job's
    position in the routing order (from 1) when the session orders the
    jobs, which it does when repair is enabled.
    """

    net_id: int
    route_index: int
    net_name: str
    inst1: str
    port1: str
    inst2: str
    port2: str
    source_port: Port
    target_port: Port


class EndpointCorrectionCategory(enum.Enum):
    """Which of the endpoint-correction passes, if any, treats a given net.

    This names the classification that was previously implicit in three
    separate, duplicated guard-clause chains across
    ``_apply_checked_endpoint_corrections_for_net_ids``,
    ``_apply_checked_fanout_stub_endpoint_corrections_for_net_ids``, and
    ``_apply_crossing_aware_endpoint_corrections_for_net_ids`` in
    ``python/photonic_router/routing/route_rust.py``. See
    ``.agent/execplans/2026-08-19-restructure-port-endpoint-correction.md``
    for the full classification table this enum names.
    """

    ALREADY_CORRECTED_NO_OP = "already_corrected_no_op"
    """Both the source and target sides are fanout stubs (pre-corrected
    endpoint adapters) and the net has no crossing. No pass touches this
    net today: it keeps exactly the centerline the earlier fanout-stub
    pre-correction stage already gave it."""

    UNRESTRICTED = "unrestricted"
    """Plain net: no fanout stub on either side, no crossing. Gets the
    full, unrestricted checked corrector on both ends."""

    FANOUT_STUB_SOURCE_ONLY = "fanout_stub_source_only"
    """Fanout stub on the source side only, no crossing. Gets the checked
    corrector on the target side only."""

    FANOUT_STUB_TARGET_ONLY = "fanout_stub_target_only"
    """Fanout stub on the target side only, no crossing. Gets the checked
    corrector on the source side only."""

    CROSSING_AWARE = "crossing_aware"
    """Net is involved in a crossing (regardless of fanout-stub status on
    either side). Handled entirely by the crossing-aware, splice-based
    pass instead of the unrestricted checked corrector."""


@dataclass(frozen=True)
class NetEndpointCorrectionClassification:
    """The result of classifying one net for endpoint correction.

    ``has_crossing`` reflects the same crossing-net-id source
    ``_apply_checked_endpoint_corrections_for_net_ids`` and
    ``_apply_checked_fanout_stub_endpoint_corrections_for_net_ids`` already
    shared (derived from ``router.crossing_events()``). The crossing-aware
    pass (pass 3) uses a second, independently-derived source
    (``_current_crossing_points_by_net_id()``) for its own net-id
    membership test; this dataclass does not merge the two, since they can
    carry different information (event membership vs. actual crossing
    points) and Milestone 1 only unifies the identical duplication between
    passes 1 and 2, not the different concept pass 3 uses.
    """

    category: EndpointCorrectionCategory
    has_crossing: bool
    source_has_fanout_stub: bool
    target_has_fanout_stub: bool


@dataclass(frozen=True)
class RipupRerouteConfig:
    """Whether route repair runs at all, and the A* history prices it uses.

    Resolved 2026-08-25 (`.agent/execplans/2026-08-25-negotiated-repair-engine.md`
    Milestone 6): `enabled` is not really a general-purpose user choice --
    every benchmark needs repair to run (even `benes_4x4`/`benes_8x8` fail
    immediately without it), confirmed unchanged by that plan's Milestone 5/6
    work. Its one legitimate use is isolating repair *out* for a diagnostic/
    test baseline (e.g. `test_benchmarks_route_with_astar_only`, which
    deliberately wants to see what plain A* alone can do). It is kept as a
    real config field rather than replaced with an explicit test-only bypass.

    Since Milestone 8 of
    `.agent/execplans/2026-09-22-modular-readable-router-restructure.md` the
    negotiated-congestion loop
    (`route_many_with_negotiated_repair_and_commit`) IS the sole repair
    mechanism -- the dispatch-chain repair this config was written for is
    gone, and with it the two round/victim budgets it owned (a per-net
    repair-round cap and a per-round victim cap, removed in Slice C
    together with their two command-line flags). The negotiated loop's
    own round budget is `NEGOTIATED_MAX_ROUNDS`
    (`python/photonic_router/routing/dispatch.py`); `enabled=False` selects the
    no-repair mode (that loop with one round and `no_ripup=True`).
    """

    enabled: bool = True
    history_weight: float = 2.0
    history_increment: int = 1


#: `RouteTimingBucket.record_route`: (bucket field, statistic of the kernel's
#: route) pairs that are summed over the routes of a bucket.
_BUCKET_SUMMED_STATS: tuple[tuple[str, str], ...] = (
    ("expanded_states", "expanded_states"),
    ("generated_neighbors", "generated_neighbors"),
    ("heap_pushes", "heap_pushes"),
    ("heap_pops", "heap_pops"),
    ("skipped_duplicate_heap_entries", "skipped_duplicate_heap_entries"),
    ("stale_generation_heap_entries", "stale_generation_heap_entries"),
    ("closed_heap_entries", "closed_heap_entries"),
    ("dense_search_states", "dense_search_states"),
    ("dense_search_storage_bytes", "dense_search_storage_bytes"),
    ("best_cost_updates", "best_cost_updates"),
    ("parent_updates", "parent_updates"),
    ("obstacle_clearance_checks", "obstacle_clearance_checks"),
    ("window_attempts", "window_attempts"),
    ("window_rejects", "window_rejects"),
    ("footprint_rejects", "footprint_rejects"),
    ("footprint_checks", "primitive_footprint_checks"),
    ("footprint_cells_tested", "primitive_footprint_cells_tested"),
    ("footprint_rect_checks", "primitive_footprint_rect_checks"),
    ("footprint_rect_rejects", "primitive_footprint_rect_rejects"),
    ("crossing_hotpath_no_contact", "crossing_hotpath_no_contact"),
    ("crossing_hotpath_contact_checks", "crossing_hotpath_contact_checks"),
    ("crossing_hotpath_static_rejects", "crossing_hotpath_static_rejects"),
    ("crossing_hotpath_no_owner_contacts", "crossing_hotpath_no_owner_contacts"),
    ("crossing_hotpath_single_owner_contacts", "crossing_hotpath_single_owner_contacts"),
    ("crossing_hotpath_multi_owner_contacts", "crossing_hotpath_multi_owner_contacts"),
    ("crossing_hotpath_witness_cells_scanned", "crossing_hotpath_witness_cells_scanned"),
    ("crossing_hotpath_partner_segment_checks", "crossing_hotpath_partner_segment_checks"),
    (
        "crossing_hotpath_partner_segment_bbox_rejects",
        "crossing_hotpath_partner_segment_bbox_rejects",
    ),
    ("crossing_hotpath_intersection_hits", "crossing_hotpath_intersection_hits"),
    ("crossing_hotpath_total_time_us", "crossing_hotpath_total_time_us"),
    ("crossing_hotpath_owner_scan_time_us", "crossing_hotpath_owner_scan_time_us"),
    ("crossing_hotpath_segment_time_us", "crossing_hotpath_segment_time_us"),
    ("crossing_hotpath_reservation_time_us", "crossing_hotpath_reservation_time_us"),
    ("crossing_candidate_checks", "crossing_candidate_checks"),
    ("crossing_accepted", "crossing_accepted"),
    ("crossing_accepted_planned", "crossing_accepted_planned"),
    ("crossing_accepted_over_budget", "crossing_accepted_over_budget"),
    ("crossing_reject_non_straight", "crossing_reject_non_straight"),
    ("crossing_reject_not_perpendicular", "crossing_reject_not_perpendicular"),
    ("crossing_reject_margin", "crossing_reject_margin"),
    ("crossing_reject_wrong_order", "crossing_reject_wrong_order"),
    ("crossing_reject_unexpected_owner", "crossing_reject_unexpected_owner"),
    ("crossing_reject_unmatched_owner", "crossing_reject_unmatched_owner"),
    ("crossing_reject_unmatched_centerline", "crossing_reject_unmatched_centerline"),
    ("crossing_reject_unmatched_footprint", "crossing_reject_unmatched_footprint"),
    ("crossing_reject_unmatched_route_centerline", "crossing_reject_unmatched_route_centerline"),
    ("crossing_reject_unmatched_route_footprint", "crossing_reject_unmatched_route_footprint"),
    ("crossing_reject_pending_straight", "crossing_reject_pending_straight"),
    ("dense_grid_build_failures", "dense_grid_build_failures"),
    ("dense_grid_cells", "dense_grid_cells"),
    ("route_search_total_time_us", "route_search_total_time_us"),
    ("dense_grid_build_time_us", "dense_grid_build_time_us"),
    ("search_loop_time_us", "search_loop_time_us"),
    ("obstacle_map_prepare_time_us", "obstacle_map_prepare_time_us"),
    ("simple_route_time_us", "simple_route_time_us"),
    ("commit_prepare_time_us", "commit_prepare_time_us"),
    ("commit_time_us", "commit_time_us"),
    ("neighbor_generation_time_us", "neighbor_generation_time_us"),
    ("heap_operation_time_us", "heap_operation_time_us"),
    ("legality_check_time_us", "legality_check_time_us"),
    ("reconstruction_time_us", "reconstruction_time_us"),
)

#: Statistics of which a bucket keeps the maximum; field and statistic share the name.
_BUCKET_MAX_STATS = (
    "max_heap_size",
    "max_window_area_cells",
)

#: Per-class counter dicts that are added key by key; field and statistic share the name.
_BUCKET_COUNTER_DICTS = (
    "primitive_generated_by_class",
    "primitive_bounds_rejects_by_class",
    "primitive_closed_rejects_by_class",
    "primitive_cost_pruned_by_class",
    "primitive_footprint_checks_by_class",
    "primitive_footprint_rejects_by_class",
    "primitive_accepted_by_class",
)


@dataclass
class RouteTimingBucket:
    """The accumulated times and search statistics of the route attempts of one kind.

    A session keeps one bucket per kind of attempt (``normal_route``,
    ``probe_route``, ``braid_ripup``, ``endpoint_correction``);
    ``summarize_route_search`` adds them up. ``calls`` and ``failures`` count
    the attempts and ``elapsed_s`` is their wall time in seconds. Every other
    field is taken from the kernel's route objects of the attempts that did
    not fail: counters are summed, ``max_heap_size`` and
    ``max_window_area_cells`` are maxima, the ``*_by_class`` dicts count per
    primitive class of ``PRIMITIVE_TRANSITION_CLASSES``, and the ``*_us``
    fields are times in microseconds.
    """

    calls: int = 0
    failures: int = 0
    elapsed_s: float = 0.0
    expanded_states: int = 0
    generated_neighbors: int = 0
    heap_pushes: int = 0
    heap_pops: int = 0
    skipped_duplicate_heap_entries: int = 0
    stale_generation_heap_entries: int = 0
    closed_heap_entries: int = 0
    max_heap_size: int = 0
    dense_search_states: int = 0
    dense_search_storage_bytes: int = 0
    best_cost_updates: int = 0
    parent_updates: int = 0
    obstacle_clearance_checks: int = 0
    window_attempts: int = 0
    window_rejects: int = 0
    footprint_rejects: int = 0
    primitive_generated_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_bounds_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_closed_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_cost_pruned_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_checks_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_accepted_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    footprint_checks: int = 0
    footprint_cells_tested: int = 0
    footprint_rect_checks: int = 0
    footprint_rect_rejects: int = 0
    crossing_hotpath_no_contact: int = 0
    crossing_hotpath_contact_checks: int = 0
    crossing_hotpath_static_rejects: int = 0
    crossing_hotpath_no_owner_contacts: int = 0
    crossing_hotpath_single_owner_contacts: int = 0
    crossing_hotpath_multi_owner_contacts: int = 0
    crossing_hotpath_witness_cells_scanned: int = 0
    crossing_hotpath_partner_segment_checks: int = 0
    crossing_hotpath_partner_segment_bbox_rejects: int = 0
    crossing_hotpath_intersection_hits: int = 0
    crossing_hotpath_total_time_us: int = 0
    crossing_hotpath_owner_scan_time_us: int = 0
    crossing_hotpath_segment_time_us: int = 0
    crossing_hotpath_reservation_time_us: int = 0
    crossing_candidate_checks: int = 0
    crossing_accepted: int = 0
    crossing_accepted_planned: int = 0
    crossing_accepted_over_budget: int = 0
    crossing_reject_non_straight: int = 0
    crossing_reject_not_perpendicular: int = 0
    crossing_reject_margin: int = 0
    crossing_reject_wrong_order: int = 0
    crossing_reject_unexpected_owner: int = 0
    crossing_reject_unmatched_owner: int = 0
    crossing_reject_unmatched_centerline: int = 0
    crossing_reject_unmatched_footprint: int = 0
    crossing_reject_unmatched_route_centerline: int = 0
    crossing_reject_unmatched_route_footprint: int = 0
    crossing_reject_pending_straight: int = 0
    dense_grid_build_failures: int = 0
    dense_grid_cells: int = 0
    route_search_total_time_us: int = 0
    dense_grid_build_time_us: int = 0
    search_loop_time_us: int = 0
    obstacle_map_prepare_time_us: int = 0
    simple_route_time_us: int = 0
    commit_prepare_time_us: int = 0
    commit_time_us: int = 0
    neighbor_generation_time_us: int = 0
    heap_operation_time_us: int = 0
    legality_check_time_us: int = 0
    reconstruction_time_us: int = 0
    max_window_area_cells: int = 0
    full_grid_fallbacks: int = 0

    @property
    def successes(self) -> int:
        """The number of recorded attempts that did not fail."""
        return self.calls - self.failures

    def record_elapsed(self, elapsed_s: float, *, failed: bool = False) -> None:
        """Count one attempt and add its wall time in seconds, in place.

        ``failed`` counts it as a failure too. The search statistics are left
        alone; ``record_route`` adds them.
        """
        self.calls += 1
        if failed:
            self.failures += 1
        self.elapsed_s += elapsed_s

    def record_route(self, elapsed_s: float, route_obj: object, *, failed: bool = False) -> None:
        """Record one attempt and, unless it failed, add its route's search statistics.

        ``route_obj`` is the kernel's route of the attempt. A statistic it
        does not have, or a negative one, counts as 0. Mutates the bucket:
        counters are summed, ``max_heap_size`` and ``max_window_area_cells``
        keep the maximum, and a route that used the full-grid fallback
        increments ``full_grid_fallbacks``.
        """
        self.record_elapsed(elapsed_s, failed=failed)
        if failed:
            return
        for field_name, stat_name in _BUCKET_SUMMED_STATS:
            setattr(
                self,
                field_name,
                getattr(self, field_name) + _get_route_int_stat(route_obj, stat_name),
            )
        for name in _BUCKET_MAX_STATS:
            setattr(self, name, max(getattr(self, name), _get_route_int_stat(route_obj, name)))
        for name in _BUCKET_COUNTER_DICTS:
            _add_counter_dict(getattr(self, name), _get_route_counter_dict(route_obj, name))
        if bool(getattr(route_obj, "used_full_grid_fallback", False)):
            self.full_grid_fallbacks += 1


@dataclass(frozen=True)
class RouteAttemptRecord:
    """One route attempt of a run, with the search statistics of that attempt alone.

    Recorded only when the run collects timing. ``attempt_index`` is the
    attempt's number in the run and ``bucket_name`` the timing bucket it was
    counted in. ``net_id`` and ``route_index`` are those of the ``RouteJob``;
    ``source`` and ``target`` are its ports as ``"instance,port"``.
    ``elapsed_s`` is the attempt's wall time in seconds and ``repair_round``
    the repair round it belongs to, when there is one. ``total_length_um``
    is None when the attempt has no route, and ``route_cells`` is the number
    of cells of the route. The remaining fields are the kernel's statistics
    of the route (0 for a failed attempt); ``*_us`` fields are microseconds.
    """

    attempt_index: int
    bucket_name: str
    net_id: int
    route_index: int
    net_name: str
    source: str
    target: str
    elapsed_s: float
    failed: bool = False
    repair_round: int | None = None
    error: str | None = None
    total_length_um: float | None = None
    route_cells: int = 0
    expanded_states: int = 0
    generated_neighbors: int = 0
    heap_pushes: int = 0
    heap_pops: int = 0
    skipped_duplicate_heap_entries: int = 0
    stale_generation_heap_entries: int = 0
    closed_heap_entries: int = 0
    max_heap_size: int = 0
    dense_search_states: int = 0
    dense_search_storage_bytes: int = 0
    best_cost_updates: int = 0
    parent_updates: int = 0
    obstacle_clearance_checks: int = 0
    window_attempts: int = 0
    last_window_min_x: int = 0
    last_window_max_x: int = 0
    last_window_min_y: int = 0
    last_window_max_y: int = 0
    last_window_area_cells: int = 0
    primitive_generated_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_bounds_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_closed_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_cost_pruned_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_checks_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_accepted_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    footprint_checks: int = 0
    footprint_rect_checks: int = 0
    crossing_hotpath_no_contact: int = 0
    crossing_hotpath_contact_checks: int = 0
    crossing_hotpath_static_rejects: int = 0
    crossing_hotpath_no_owner_contacts: int = 0
    crossing_hotpath_single_owner_contacts: int = 0
    crossing_hotpath_multi_owner_contacts: int = 0
    crossing_hotpath_witness_cells_scanned: int = 0
    crossing_hotpath_partner_segment_checks: int = 0
    crossing_hotpath_partner_segment_bbox_rejects: int = 0
    crossing_hotpath_intersection_hits: int = 0
    crossing_hotpath_total_time_us: int = 0
    crossing_hotpath_owner_scan_time_us: int = 0
    crossing_hotpath_segment_time_us: int = 0
    crossing_hotpath_reservation_time_us: int = 0
    crossing_candidate_checks: int = 0
    crossing_accepted: int = 0
    crossing_accepted_planned: int = 0
    crossing_accepted_over_budget: int = 0
    crossing_reject_non_straight: int = 0
    crossing_reject_not_perpendicular: int = 0
    crossing_reject_margin: int = 0
    crossing_reject_wrong_order: int = 0
    crossing_reject_unexpected_owner: int = 0
    crossing_reject_unmatched_owner: int = 0
    crossing_reject_unmatched_centerline: int = 0
    crossing_reject_unmatched_footprint: int = 0
    crossing_reject_unmatched_route_centerline: int = 0
    crossing_reject_unmatched_route_footprint: int = 0
    crossing_reject_pending_straight: int = 0
    route_search_total_time_us: int = 0
    dense_grid_build_time_us: int = 0
    dense_grid_cells: int = 0
    search_loop_time_us: int = 0
    obstacle_map_prepare_time_us: int = 0
    simple_route_time_us: int = 0
    commit_prepare_time_us: int = 0
    commit_time_us: int = 0
    neighbor_generation_time_us: int = 0
    heap_operation_time_us: int = 0
    legality_check_time_us: int = 0
    reconstruction_time_us: int = 0
    max_window_area_cells: int = 0
    used_full_grid_fallback: bool = False
    diagnostics: dict[str, object] = field(default_factory=dict)

    @property
    def used_simple_route(self) -> bool:
        """Whether the attempt succeeded without expanding a single search state."""
        return not self.failed and self.expanded_states == 0

    def as_dict(self) -> dict[str, object]:
        """The record as a flat dict, with every time in seconds.

        Each field is reported under its own name, except that a ``*_us``
        field (microseconds) is reported in seconds as ``*_s``.
        ``used_simple_route`` is included; the per-class counters and the
        diagnostics are copies.
        """
        return {
            "attempt_index": self.attempt_index,
            "bucket_name": self.bucket_name,
            "net_id": self.net_id,
            "route_index": self.route_index,
            "net_name": self.net_name,
            "source": self.source,
            "target": self.target,
            "elapsed_s": self.elapsed_s,
            "failed": self.failed,
            "repair_round": self.repair_round,
            "error": self.error,
            "total_length_um": self.total_length_um,
            "route_cells": self.route_cells,
            "used_simple_route": self.used_simple_route,
            "expanded_states": self.expanded_states,
            "generated_neighbors": self.generated_neighbors,
            "heap_pushes": self.heap_pushes,
            "heap_pops": self.heap_pops,
            "skipped_duplicate_heap_entries": self.skipped_duplicate_heap_entries,
            "stale_generation_heap_entries": self.stale_generation_heap_entries,
            "closed_heap_entries": self.closed_heap_entries,
            "max_heap_size": self.max_heap_size,
            "dense_search_states": self.dense_search_states,
            "dense_search_storage_bytes": self.dense_search_storage_bytes,
            "best_cost_updates": self.best_cost_updates,
            "parent_updates": self.parent_updates,
            "obstacle_clearance_checks": self.obstacle_clearance_checks,
            "window_attempts": self.window_attempts,
            "last_window_min_x": self.last_window_min_x,
            "last_window_max_x": self.last_window_max_x,
            "last_window_min_y": self.last_window_min_y,
            "last_window_max_y": self.last_window_max_y,
            "last_window_area_cells": self.last_window_area_cells,
            "primitive_generated_by_class": dict(self.primitive_generated_by_class),
            "primitive_bounds_rejects_by_class": dict(self.primitive_bounds_rejects_by_class),
            "primitive_closed_rejects_by_class": dict(self.primitive_closed_rejects_by_class),
            "primitive_cost_pruned_by_class": dict(self.primitive_cost_pruned_by_class),
            "primitive_footprint_checks_by_class": dict(self.primitive_footprint_checks_by_class),
            "primitive_footprint_rejects_by_class": dict(self.primitive_footprint_rejects_by_class),
            "primitive_accepted_by_class": dict(self.primitive_accepted_by_class),
            "footprint_checks": self.footprint_checks,
            "footprint_rect_checks": self.footprint_rect_checks,
            "crossing_hotpath_no_contact": self.crossing_hotpath_no_contact,
            "crossing_hotpath_contact_checks": self.crossing_hotpath_contact_checks,
            "crossing_hotpath_static_rejects": self.crossing_hotpath_static_rejects,
            "crossing_hotpath_no_owner_contacts": self.crossing_hotpath_no_owner_contacts,
            "crossing_hotpath_single_owner_contacts": self.crossing_hotpath_single_owner_contacts,
            "crossing_hotpath_multi_owner_contacts": self.crossing_hotpath_multi_owner_contacts,
            "crossing_hotpath_witness_cells_scanned": self.crossing_hotpath_witness_cells_scanned,
            "crossing_hotpath_partner_segment_checks": self.crossing_hotpath_partner_segment_checks,
            "crossing_hotpath_partner_segment_bbox_rejects": self.crossing_hotpath_partner_segment_bbox_rejects,
            "crossing_hotpath_intersection_hits": self.crossing_hotpath_intersection_hits,
            "crossing_hotpath_total_time_s": self.crossing_hotpath_total_time_us / 1_000_000.0,
            "crossing_hotpath_owner_scan_time_s": self.crossing_hotpath_owner_scan_time_us
            / 1_000_000.0,
            "crossing_hotpath_segment_time_s": self.crossing_hotpath_segment_time_us / 1_000_000.0,
            "crossing_hotpath_reservation_time_s": self.crossing_hotpath_reservation_time_us
            / 1_000_000.0,
            "crossing_candidate_checks": self.crossing_candidate_checks,
            "crossing_accepted": self.crossing_accepted,
            "crossing_accepted_planned": self.crossing_accepted_planned,
            "crossing_accepted_over_budget": self.crossing_accepted_over_budget,
            "crossing_reject_non_straight": self.crossing_reject_non_straight,
            "crossing_reject_not_perpendicular": (self.crossing_reject_not_perpendicular),
            "crossing_reject_margin": self.crossing_reject_margin,
            "crossing_reject_wrong_order": self.crossing_reject_wrong_order,
            "crossing_reject_unexpected_owner": (self.crossing_reject_unexpected_owner),
            "crossing_reject_unmatched_owner": self.crossing_reject_unmatched_owner,
            "crossing_reject_unmatched_centerline": (self.crossing_reject_unmatched_centerline),
            "crossing_reject_unmatched_footprint": (self.crossing_reject_unmatched_footprint),
            "crossing_reject_unmatched_route_centerline": (
                self.crossing_reject_unmatched_route_centerline
            ),
            "crossing_reject_unmatched_route_footprint": (
                self.crossing_reject_unmatched_route_footprint
            ),
            "crossing_reject_pending_straight": (self.crossing_reject_pending_straight),
            "route_search_total_time_s": (self.route_search_total_time_us / 1_000_000.0),
            "dense_grid_build_time_s": self.dense_grid_build_time_us / 1_000_000.0,
            "dense_grid_cells": self.dense_grid_cells,
            "search_loop_time_s": self.search_loop_time_us / 1_000_000.0,
            "obstacle_map_prepare_time_s": (self.obstacle_map_prepare_time_us / 1_000_000.0),
            "simple_route_time_s": self.simple_route_time_us / 1_000_000.0,
            "commit_prepare_time_s": self.commit_prepare_time_us / 1_000_000.0,
            "commit_time_s": self.commit_time_us / 1_000_000.0,
            "neighbor_generation_time_s": self.neighbor_generation_time_us / 1_000_000.0,
            "heap_operation_time_s": self.heap_operation_time_us / 1_000_000.0,
            "legality_check_time_s": self.legality_check_time_us / 1_000_000.0,
            "reconstruction_time_s": self.reconstruction_time_us / 1_000_000.0,
            "max_window_area_cells": self.max_window_area_cells,
            "used_full_grid_fallback": self.used_full_grid_fallback,
            "diagnostics": dict(self.diagnostics),
        }


def route_attempt_record_from_route(
    *,
    attempt_index: int,
    bucket_name: str,
    net_id: int,
    route_index: int,
    net_name: str,
    source: str,
    target: str,
    elapsed_s: float,
    route_obj: object | None = None,
    failed: bool = False,
    repair_round: int | None = None,
    error: str | None = None,
    diagnostics: Mapping[str, object] | None = None,
) -> RouteAttemptRecord:
    """Build the record of one route attempt from the kernel's route object.

    The identifying arguments are stored as given (``source`` and ``target``
    are ``"instance,port"`` strings, ``elapsed_s`` is in seconds). Every
    search statistic is read from the attribute of the same name on
    ``route_obj``; a missing attribute, and ``route_obj=None`` (an attempt
    without a route), give 0. ``total_length_um`` is None when the route
    object has no such attribute, and ``route_cells`` is the length of its
    ``cells``. ``diagnostics`` is copied.
    """
    route_cells = getattr(route_obj, "cells", None) if route_obj is not None else None
    total_length_um = (
        as_float(route_obj.total_length_um, 0.0)
        if route_obj is not None and hasattr(route_obj, "total_length_um")
        else None
    )
    return RouteAttemptRecord(
        attempt_index=int(attempt_index),
        bucket_name=bucket_name,
        net_id=int(net_id),
        route_index=int(route_index),
        net_name=net_name,
        source=source,
        target=target,
        elapsed_s=float(elapsed_s),
        failed=bool(failed),
        repair_round=repair_round,
        error=error,
        total_length_um=total_length_um,
        route_cells=len(route_cells or ()),
        expanded_states=_get_route_int_stat(route_obj, "expanded_states"),
        generated_neighbors=_get_route_int_stat(route_obj, "generated_neighbors"),
        heap_pushes=_get_route_int_stat(route_obj, "heap_pushes"),
        heap_pops=_get_route_int_stat(route_obj, "heap_pops"),
        skipped_duplicate_heap_entries=_get_route_int_stat(
            route_obj,
            "skipped_duplicate_heap_entries",
        ),
        stale_generation_heap_entries=_get_route_int_stat(
            route_obj,
            "stale_generation_heap_entries",
        ),
        closed_heap_entries=_get_route_int_stat(
            route_obj,
            "closed_heap_entries",
        ),
        max_heap_size=_get_route_int_stat(route_obj, "max_heap_size"),
        dense_search_states=_get_route_int_stat(route_obj, "dense_search_states"),
        dense_search_storage_bytes=_get_route_int_stat(
            route_obj,
            "dense_search_storage_bytes",
        ),
        best_cost_updates=_get_route_int_stat(route_obj, "best_cost_updates"),
        parent_updates=_get_route_int_stat(route_obj, "parent_updates"),
        obstacle_clearance_checks=_get_route_int_stat(
            route_obj,
            "obstacle_clearance_checks",
        ),
        window_attempts=_get_route_int_stat(route_obj, "window_attempts"),
        last_window_min_x=_get_route_int_stat(route_obj, "last_window_min_x"),
        last_window_max_x=_get_route_int_stat(route_obj, "last_window_max_x"),
        last_window_min_y=_get_route_int_stat(route_obj, "last_window_min_y"),
        last_window_max_y=_get_route_int_stat(route_obj, "last_window_max_y"),
        last_window_area_cells=_get_route_int_stat(route_obj, "last_window_area_cells"),
        primitive_generated_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_generated_by_class",
        ),
        primitive_bounds_rejects_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_bounds_rejects_by_class",
        ),
        primitive_closed_rejects_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_closed_rejects_by_class",
        ),
        primitive_cost_pruned_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_cost_pruned_by_class",
        ),
        primitive_footprint_checks_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_footprint_checks_by_class",
        ),
        primitive_footprint_rejects_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_footprint_rejects_by_class",
        ),
        primitive_accepted_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_accepted_by_class",
        ),
        footprint_checks=_get_route_int_stat(route_obj, "primitive_footprint_checks"),
        footprint_rect_checks=_get_route_int_stat(
            route_obj,
            "primitive_footprint_rect_checks",
        ),
        crossing_hotpath_no_contact=_get_route_int_stat(route_obj, "crossing_hotpath_no_contact"),
        crossing_hotpath_contact_checks=_get_route_int_stat(
            route_obj, "crossing_hotpath_contact_checks"
        ),
        crossing_hotpath_static_rejects=_get_route_int_stat(
            route_obj, "crossing_hotpath_static_rejects"
        ),
        crossing_hotpath_no_owner_contacts=_get_route_int_stat(
            route_obj, "crossing_hotpath_no_owner_contacts"
        ),
        crossing_hotpath_single_owner_contacts=_get_route_int_stat(
            route_obj, "crossing_hotpath_single_owner_contacts"
        ),
        crossing_hotpath_multi_owner_contacts=_get_route_int_stat(
            route_obj, "crossing_hotpath_multi_owner_contacts"
        ),
        crossing_hotpath_witness_cells_scanned=_get_route_int_stat(
            route_obj, "crossing_hotpath_witness_cells_scanned"
        ),
        crossing_hotpath_partner_segment_checks=_get_route_int_stat(
            route_obj, "crossing_hotpath_partner_segment_checks"
        ),
        crossing_hotpath_partner_segment_bbox_rejects=_get_route_int_stat(
            route_obj, "crossing_hotpath_partner_segment_bbox_rejects"
        ),
        crossing_hotpath_intersection_hits=_get_route_int_stat(
            route_obj, "crossing_hotpath_intersection_hits"
        ),
        crossing_hotpath_total_time_us=_get_route_int_stat(
            route_obj, "crossing_hotpath_total_time_us"
        ),
        crossing_hotpath_owner_scan_time_us=_get_route_int_stat(
            route_obj, "crossing_hotpath_owner_scan_time_us"
        ),
        crossing_hotpath_segment_time_us=_get_route_int_stat(
            route_obj, "crossing_hotpath_segment_time_us"
        ),
        crossing_hotpath_reservation_time_us=_get_route_int_stat(
            route_obj, "crossing_hotpath_reservation_time_us"
        ),
        crossing_candidate_checks=_get_route_int_stat(route_obj, "crossing_candidate_checks"),
        crossing_accepted=_get_route_int_stat(route_obj, "crossing_accepted"),
        crossing_accepted_planned=_get_route_int_stat(route_obj, "crossing_accepted_planned"),
        crossing_accepted_over_budget=_get_route_int_stat(
            route_obj, "crossing_accepted_over_budget"
        ),
        crossing_reject_non_straight=_get_route_int_stat(route_obj, "crossing_reject_non_straight"),
        crossing_reject_not_perpendicular=_get_route_int_stat(
            route_obj, "crossing_reject_not_perpendicular"
        ),
        crossing_reject_margin=_get_route_int_stat(route_obj, "crossing_reject_margin"),
        crossing_reject_wrong_order=_get_route_int_stat(route_obj, "crossing_reject_wrong_order"),
        crossing_reject_unexpected_owner=_get_route_int_stat(
            route_obj, "crossing_reject_unexpected_owner"
        ),
        crossing_reject_unmatched_owner=_get_route_int_stat(
            route_obj, "crossing_reject_unmatched_owner"
        ),
        crossing_reject_unmatched_centerline=_get_route_int_stat(
            route_obj, "crossing_reject_unmatched_centerline"
        ),
        crossing_reject_unmatched_footprint=_get_route_int_stat(
            route_obj, "crossing_reject_unmatched_footprint"
        ),
        crossing_reject_unmatched_route_centerline=_get_route_int_stat(
            route_obj, "crossing_reject_unmatched_route_centerline"
        ),
        crossing_reject_unmatched_route_footprint=_get_route_int_stat(
            route_obj, "crossing_reject_unmatched_route_footprint"
        ),
        crossing_reject_pending_straight=_get_route_int_stat(
            route_obj, "crossing_reject_pending_straight"
        ),
        route_search_total_time_us=_get_route_int_stat(
            route_obj,
            "route_search_total_time_us",
        ),
        dense_grid_build_time_us=_get_route_int_stat(
            route_obj,
            "dense_grid_build_time_us",
        ),
        dense_grid_cells=_get_route_int_stat(route_obj, "dense_grid_cells"),
        search_loop_time_us=_get_route_int_stat(
            route_obj,
            "search_loop_time_us",
        ),
        obstacle_map_prepare_time_us=_get_route_int_stat(
            route_obj,
            "obstacle_map_prepare_time_us",
        ),
        simple_route_time_us=_get_route_int_stat(
            route_obj,
            "simple_route_time_us",
        ),
        commit_prepare_time_us=_get_route_int_stat(
            route_obj,
            "commit_prepare_time_us",
        ),
        commit_time_us=_get_route_int_stat(
            route_obj,
            "commit_time_us",
        ),
        neighbor_generation_time_us=_get_route_int_stat(
            route_obj,
            "neighbor_generation_time_us",
        ),
        heap_operation_time_us=_get_route_int_stat(
            route_obj,
            "heap_operation_time_us",
        ),
        legality_check_time_us=_get_route_int_stat(
            route_obj,
            "legality_check_time_us",
        ),
        reconstruction_time_us=_get_route_int_stat(
            route_obj,
            "reconstruction_time_us",
        ),
        max_window_area_cells=_get_route_int_stat(route_obj, "max_window_area_cells"),
        used_full_grid_fallback=bool(getattr(route_obj, "used_full_grid_fallback", False)),
        diagnostics=dict(diagnostics or {}),
    )


@dataclass(frozen=True)
class RouteSearchSummary:
    """The totals of a run's route search, as ``summarize_route_search`` builds them.

    ``route_attempts``, ``route_failures`` and the search statistics are
    taken over every timing bucket except ``endpoint_correction``: counters
    are summed, ``max_*`` fields are maxima. The endpoint corrections are
    reported in the three ``endpoint_correction_*`` fields, and
    ``normal_route_time_s`` is the time of the ``normal_route`` bucket
    alone. ``route_count``, ``simple_route_count``, ``repair_count`` and
    ``astar_elapsed_s`` are the session's own counts and its search time.
    Times are in seconds, except the ``*_us`` fields in microseconds.
    """

    route_count: int = 0
    route_attempts: int = 0
    route_failures: int = 0
    simple_route_count: int = 0
    repair_count: int = 0
    astar_elapsed_s: float = 0.0
    endpoint_correction_time_s: float = 0.0
    endpoint_correction_calls: int = 0
    endpoint_correction_failures: int = 0
    normal_route_time_s: float = 0.0
    expanded_states: int = 0
    generated_neighbors: int = 0
    heap_pushes: int = 0
    heap_pops: int = 0
    skipped_duplicate_heap_entries: int = 0
    stale_generation_heap_entries: int = 0
    closed_heap_entries: int = 0
    max_heap_size: int = 0
    dense_search_states: int = 0
    dense_search_storage_bytes: int = 0
    best_cost_updates: int = 0
    parent_updates: int = 0
    obstacle_clearance_checks: int = 0
    primitive_generated_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_bounds_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_closed_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_cost_pruned_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_checks_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_accepted_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    footprint_checks: int = 0
    footprint_rejects: int = 0
    footprint_rect_checks: int = 0
    footprint_rect_rejects: int = 0
    crossing_hotpath_no_contact: int = 0
    crossing_hotpath_contact_checks: int = 0
    crossing_hotpath_static_rejects: int = 0
    crossing_hotpath_no_owner_contacts: int = 0
    crossing_hotpath_single_owner_contacts: int = 0
    crossing_hotpath_multi_owner_contacts: int = 0
    crossing_hotpath_witness_cells_scanned: int = 0
    crossing_hotpath_partner_segment_checks: int = 0
    crossing_hotpath_partner_segment_bbox_rejects: int = 0
    crossing_hotpath_intersection_hits: int = 0
    crossing_hotpath_total_time_us: int = 0
    crossing_hotpath_owner_scan_time_us: int = 0
    crossing_hotpath_segment_time_us: int = 0
    crossing_hotpath_reservation_time_us: int = 0
    crossing_candidate_checks: int = 0
    crossing_accepted: int = 0
    crossing_accepted_planned: int = 0
    crossing_accepted_over_budget: int = 0
    crossing_reject_non_straight: int = 0
    crossing_reject_not_perpendicular: int = 0
    crossing_reject_margin: int = 0
    crossing_reject_wrong_order: int = 0
    crossing_reject_unexpected_owner: int = 0
    crossing_reject_unmatched_owner: int = 0
    crossing_reject_unmatched_centerline: int = 0
    crossing_reject_unmatched_footprint: int = 0
    crossing_reject_unmatched_route_centerline: int = 0
    crossing_reject_unmatched_route_footprint: int = 0
    crossing_reject_pending_straight: int = 0
    route_search_total_time_us: int = 0
    dense_grid_build_time_us: int = 0
    dense_grid_cells: int = 0
    search_loop_time_us: int = 0
    obstacle_map_prepare_time_us: int = 0
    simple_route_time_us: int = 0
    commit_prepare_time_us: int = 0
    commit_time_us: int = 0
    neighbor_generation_time_us: int = 0
    heap_operation_time_us: int = 0
    legality_check_time_us: int = 0
    reconstruction_time_us: int = 0
    max_window_area_cells: int = 0
    full_grid_fallbacks: int = 0


def summarize_route_search(
    route_timing_buckets: Mapping[str, RouteTimingBucket],
    *,
    route_count: int,
    simple_route_count: int,
    repair_count: int,
    astar_elapsed_s: float,
) -> RouteSearchSummary:
    """Aggregate route-search counters used by quiet benchmark reporting."""
    non_route_bucket_names = {"endpoint_correction"}
    buckets = [
        bucket
        for bucket_name, bucket in route_timing_buckets.items()
        if bucket_name not in non_route_bucket_names
    ]

    def sum_bucket_counters(attr: str) -> dict[str, int]:
        counters = _empty_primitive_counter_dict()
        for bucket in buckets:
            _add_counter_dict(counters, getattr(bucket, attr))
        return counters

    return RouteSearchSummary(
        route_count=int(route_count),
        route_attempts=sum(bucket.calls for bucket in buckets),
        route_failures=sum(bucket.failures for bucket in buckets),
        simple_route_count=int(simple_route_count),
        repair_count=int(repair_count),
        astar_elapsed_s=float(astar_elapsed_s),
        endpoint_correction_time_s=float(
            route_timing_buckets.get("endpoint_correction", RouteTimingBucket()).elapsed_s
        ),
        endpoint_correction_calls=int(
            route_timing_buckets.get("endpoint_correction", RouteTimingBucket()).calls
        ),
        endpoint_correction_failures=int(
            route_timing_buckets.get("endpoint_correction", RouteTimingBucket()).failures
        ),
        normal_route_time_s=float(
            route_timing_buckets.get("normal_route", RouteTimingBucket()).elapsed_s
        ),
        expanded_states=sum(bucket.expanded_states for bucket in buckets),
        generated_neighbors=sum(bucket.generated_neighbors for bucket in buckets),
        heap_pushes=sum(bucket.heap_pushes for bucket in buckets),
        heap_pops=sum(bucket.heap_pops for bucket in buckets),
        skipped_duplicate_heap_entries=sum(
            bucket.skipped_duplicate_heap_entries for bucket in buckets
        ),
        stale_generation_heap_entries=sum(
            bucket.stale_generation_heap_entries for bucket in buckets
        ),
        closed_heap_entries=sum(bucket.closed_heap_entries for bucket in buckets),
        max_heap_size=max((bucket.max_heap_size for bucket in buckets), default=0),
        dense_search_states=sum(bucket.dense_search_states for bucket in buckets),
        dense_search_storage_bytes=sum(bucket.dense_search_storage_bytes for bucket in buckets),
        best_cost_updates=sum(bucket.best_cost_updates for bucket in buckets),
        parent_updates=sum(bucket.parent_updates for bucket in buckets),
        obstacle_clearance_checks=sum(bucket.obstacle_clearance_checks for bucket in buckets),
        primitive_generated_by_class=sum_bucket_counters("primitive_generated_by_class"),
        primitive_bounds_rejects_by_class=sum_bucket_counters("primitive_bounds_rejects_by_class"),
        primitive_closed_rejects_by_class=sum_bucket_counters("primitive_closed_rejects_by_class"),
        primitive_cost_pruned_by_class=sum_bucket_counters("primitive_cost_pruned_by_class"),
        primitive_footprint_checks_by_class=sum_bucket_counters(
            "primitive_footprint_checks_by_class"
        ),
        primitive_footprint_rejects_by_class=sum_bucket_counters(
            "primitive_footprint_rejects_by_class"
        ),
        primitive_accepted_by_class=sum_bucket_counters("primitive_accepted_by_class"),
        footprint_checks=sum(bucket.footprint_checks for bucket in buckets),
        footprint_rejects=sum(bucket.footprint_rejects for bucket in buckets),
        footprint_rect_checks=sum(bucket.footprint_rect_checks for bucket in buckets),
        footprint_rect_rejects=sum(bucket.footprint_rect_rejects for bucket in buckets),
        crossing_hotpath_no_contact=sum(bucket.crossing_hotpath_no_contact for bucket in buckets),
        crossing_hotpath_contact_checks=sum(
            bucket.crossing_hotpath_contact_checks for bucket in buckets
        ),
        crossing_hotpath_static_rejects=sum(
            bucket.crossing_hotpath_static_rejects for bucket in buckets
        ),
        crossing_hotpath_no_owner_contacts=sum(
            bucket.crossing_hotpath_no_owner_contacts for bucket in buckets
        ),
        crossing_hotpath_single_owner_contacts=sum(
            bucket.crossing_hotpath_single_owner_contacts for bucket in buckets
        ),
        crossing_hotpath_multi_owner_contacts=sum(
            bucket.crossing_hotpath_multi_owner_contacts for bucket in buckets
        ),
        crossing_hotpath_witness_cells_scanned=sum(
            bucket.crossing_hotpath_witness_cells_scanned for bucket in buckets
        ),
        crossing_hotpath_partner_segment_checks=sum(
            bucket.crossing_hotpath_partner_segment_checks for bucket in buckets
        ),
        crossing_hotpath_partner_segment_bbox_rejects=sum(
            bucket.crossing_hotpath_partner_segment_bbox_rejects for bucket in buckets
        ),
        crossing_hotpath_intersection_hits=sum(
            bucket.crossing_hotpath_intersection_hits for bucket in buckets
        ),
        crossing_hotpath_total_time_us=sum(
            bucket.crossing_hotpath_total_time_us for bucket in buckets
        ),
        crossing_hotpath_owner_scan_time_us=sum(
            bucket.crossing_hotpath_owner_scan_time_us for bucket in buckets
        ),
        crossing_hotpath_segment_time_us=sum(
            bucket.crossing_hotpath_segment_time_us for bucket in buckets
        ),
        crossing_hotpath_reservation_time_us=sum(
            bucket.crossing_hotpath_reservation_time_us for bucket in buckets
        ),
        crossing_candidate_checks=sum(bucket.crossing_candidate_checks for bucket in buckets),
        crossing_accepted=sum(bucket.crossing_accepted for bucket in buckets),
        crossing_accepted_planned=sum(bucket.crossing_accepted_planned for bucket in buckets),
        crossing_accepted_over_budget=sum(
            bucket.crossing_accepted_over_budget for bucket in buckets
        ),
        crossing_reject_non_straight=sum(bucket.crossing_reject_non_straight for bucket in buckets),
        crossing_reject_not_perpendicular=sum(
            bucket.crossing_reject_not_perpendicular for bucket in buckets
        ),
        crossing_reject_margin=sum(bucket.crossing_reject_margin for bucket in buckets),
        crossing_reject_wrong_order=sum(bucket.crossing_reject_wrong_order for bucket in buckets),
        crossing_reject_unexpected_owner=sum(
            bucket.crossing_reject_unexpected_owner for bucket in buckets
        ),
        crossing_reject_unmatched_owner=sum(
            bucket.crossing_reject_unmatched_owner for bucket in buckets
        ),
        crossing_reject_unmatched_centerline=sum(
            bucket.crossing_reject_unmatched_centerline for bucket in buckets
        ),
        crossing_reject_unmatched_footprint=sum(
            bucket.crossing_reject_unmatched_footprint for bucket in buckets
        ),
        crossing_reject_unmatched_route_centerline=sum(
            bucket.crossing_reject_unmatched_route_centerline for bucket in buckets
        ),
        crossing_reject_unmatched_route_footprint=sum(
            bucket.crossing_reject_unmatched_route_footprint for bucket in buckets
        ),
        crossing_reject_pending_straight=sum(
            bucket.crossing_reject_pending_straight for bucket in buckets
        ),
        route_search_total_time_us=sum(bucket.route_search_total_time_us for bucket in buckets),
        dense_grid_build_time_us=sum(bucket.dense_grid_build_time_us for bucket in buckets),
        dense_grid_cells=sum(bucket.dense_grid_cells for bucket in buckets),
        search_loop_time_us=sum(bucket.search_loop_time_us for bucket in buckets),
        obstacle_map_prepare_time_us=sum(bucket.obstacle_map_prepare_time_us for bucket in buckets),
        simple_route_time_us=sum(bucket.simple_route_time_us for bucket in buckets),
        commit_prepare_time_us=sum(bucket.commit_prepare_time_us for bucket in buckets),
        commit_time_us=sum(bucket.commit_time_us for bucket in buckets),
        neighbor_generation_time_us=sum(bucket.neighbor_generation_time_us for bucket in buckets),
        heap_operation_time_us=sum(bucket.heap_operation_time_us for bucket in buckets),
        legality_check_time_us=sum(bucket.legality_check_time_us for bucket in buckets),
        reconstruction_time_us=sum(bucket.reconstruction_time_us for bucket in buckets),
        max_window_area_cells=max(
            (bucket.max_window_area_cells for bucket in buckets),
            default=0,
        ),
        full_grid_fallbacks=sum(bucket.full_grid_fallbacks for bucket in buckets),
    )


DEFAULT_MEANDER_MIN_STRAIGHT_UM = 1.0


@dataclass(frozen=True)
class MeanderInsertionConfig:
    """The settings of meander insertion for path-length matching; lengths in um.

    ``min_candidate_straight_length_um`` and ``max_meander_height_um`` go
    into the kernel's meander search policy, as does
    ``auto_meander_endpoint_inset_um``: a value is a fixed endpoint inset,
    None selects the adaptive one.
    """

    min_candidate_straight_length_um: float = DEFAULT_MEANDER_MIN_STRAIGHT_UM
    max_meander_height_um: float = DEFAULT_MEANDER_MAX_HEIGHT_UM
    auto_meander_endpoint_inset_um: float | None = None


def edge_key_to_dict(edge_key: RoutedEdgeKey) -> dict[str, object]:
    """A routed edge key as a JSON-serializable dict.

    ``net_name``, and ``source`` and ``target`` each as a dict with
    ``instance`` and ``port``.
    """
    return {
        "net_name": edge_key.net_name,
        "source": {"instance": edge_key.source.instance, "port": edge_key.source.port},
        "target": {"instance": edge_key.target.instance, "port": edge_key.target.port},
    }


#: The scalar fields of a Rust meander plan the report keeps for a planned
#: requirement (its ``plan``): what the console listing and the GDS meta info
#: show per meander. The plan's centerline and candidate counts stay out.
MEANDER_PLAN_INFO_KEYS: tuple[str, ...] = (
    "planning_mode",
    "effective_bend_radius_um",
    "primitive_bend_radius_um",
    "selected_box",
    "selected_grid_rect",
    "bumps",
    "visual_bumps",
    "u_turns",
    "quarter_turns",
    "side",
    "endpoint_inset_um",
    "selected_interval_length_um",
)


@dataclass
class MeanderEdgeReport:
    """One path-length requirement's meander result (phase 6 slice B).

    ``inserted_extra_length_um`` is what the bookkeeping credits to the
    requiring ``edge``; the meander itself is drawn on ``planned_edges`` (the
    requiring edge or a delay-insertion candidate upstream of it) and
    ``physical_inserted_extra_length_um`` is its total there. A requirement
    satisfied through another requirement's candidate (a shared upstream
    bundle) carries that requirement in ``satisfied_by_requirement_edge``.
    ``plan`` is the Rust plan of the drawn meander (its scalars go into the
    info dict, see ``MEANDER_PLAN_INFO_KEYS``).
    """

    edge: RoutedEdgeKey
    requested_extra_length_um: float
    status: str
    reason: str = ""
    inserted_extra_length_um: float = 0.0
    physical_inserted_extra_length_um: float = 0.0
    unmatched_length_um: float = 0.0
    candidate_reason: str | None = None
    planned_edges: tuple[RoutedEdgeKey, ...] = ()
    satisfied_by_requirement_edge: RoutedEdgeKey | None = None
    planner_requested_extra_length_um: float = 0.0
    existing_physical_extra_length_um: float = 0.0
    plan: Mapping[str, object] | None = None

    def to_info_dict(self) -> dict[str, object]:
        """The result as a dict for the GDS meta info.

        The edges are written with ``edge_key_to_dict`` and the lengths as
        floats in um; ``candidate_reason`` appears as
        ``selected_candidate_reason``. ``satisfied_by_requirement_edge`` is
        present only when set, and of ``plan`` only the scalars named in
        ``MEANDER_PLAN_INFO_KEYS`` that the plan has.
        """
        info: dict[str, object] = {
            "edge": edge_key_to_dict(self.edge),
            "requested_extra_length_um": float(self.requested_extra_length_um),
            "status": self.status,
            "reason": self.reason,
            "inserted_extra_length_um": float(self.inserted_extra_length_um),
            "physical_inserted_extra_length_um": float(self.physical_inserted_extra_length_um),
            "unmatched_length_um": float(self.unmatched_length_um),
            "planner_requested_extra_length_um": float(self.planner_requested_extra_length_um),
            "existing_physical_extra_length_um": float(self.existing_physical_extra_length_um),
            "selected_candidate_reason": self.candidate_reason,
            "planned_edges": [edge_key_to_dict(edge_key) for edge_key in self.planned_edges],
        }
        if self.satisfied_by_requirement_edge is not None:
            info["satisfied_by_requirement_edge"] = edge_key_to_dict(
                self.satisfied_by_requirement_edge
            )
        if self.plan is not None:
            for key in MEANDER_PLAN_INFO_KEYS:
                if key in self.plan:
                    info[key] = self.plan[key]
        return info


@dataclass
class MeanderPlanningProfile:
    """Timers, counters and per-candidate attempts of one meander planning run.

    Built only under ``--debug-meanders``; nothing the acceptance decision or
    the SUMMARY line needs lives here.
    """

    planner_calls: int = 0
    planner_elapsed_s: float = 0.0
    final_planner_calls: int = 0
    final_planner_elapsed_s: float = 0.0
    commit_elapsed_s: float = 0.0
    setup_profile: dict[str, float] = field(default_factory=dict)
    selection_setup_profile: dict[str, float] = field(default_factory=dict)
    final_setup_profile: dict[str, float] = field(default_factory=dict)
    commit_profile: dict[str, float] = field(default_factory=dict)
    rust_planner_profile: dict[str, float] = field(default_factory=dict)
    rust_wrapper_profile: dict[str, float] = field(default_factory=dict)
    #: per candidate reason: attempts, planned, no_candidate, edge calls, elapsed
    candidate_profile: dict[str, dict[str, object]] = field(default_factory=dict)
    #: per requirement: the request vector Rust saw and the outcome per candidate
    requirement_attempts: list[dict[str, object]] = field(default_factory=list)
    search_config: dict[str, object] = field(default_factory=dict)

    def to_info_dict(self) -> dict[str, object]:
        """The profile as a dict keyed by field name; containers are shallow copies."""
        return {
            "planner_calls": int(self.planner_calls),
            "planner_elapsed_s": float(self.planner_elapsed_s),
            "final_planner_calls": int(self.final_planner_calls),
            "final_planner_elapsed_s": float(self.final_planner_elapsed_s),
            "commit_elapsed_s": float(self.commit_elapsed_s),
            "setup_profile": dict(self.setup_profile),
            "selection_setup_profile": dict(self.selection_setup_profile),
            "final_setup_profile": dict(self.final_setup_profile),
            "commit_profile": dict(self.commit_profile),
            "rust_planner_profile": dict(self.rust_planner_profile),
            "rust_wrapper_profile": dict(self.rust_wrapper_profile),
            "candidate_profile": dict(self.candidate_profile),
            "requirement_attempts": list(self.requirement_attempts),
            "search_config": dict(self.search_config),
        }


@dataclass
class MeanderInsertionReport:
    """The meander planning result the flow and the acceptance diagnostics read.

    ``results`` has one entry per requirement (plus one per further
    requirement a shared candidate satisfied); ``realized_extra_by_edge`` is
    the bookkeeping by physical edge (2026-10-05): the meander actually drawn
    on each edge, which the realized arrivals and spreads are computed from.
    ``to_info_dict`` is the GDS meta info (``meander_insertion_report``) and
    carries the profile only when one was built.
    """

    results: list[MeanderEdgeReport]
    total_requested_extra_length_um: float
    total_inserted_extra_length_um: float
    total_physical_inserted_extra_length_um: float
    total_disregarded_extra_length_um: float
    unmatched_length_um: float
    realized_extra_by_edge: dict[RoutedEdgeKey, float]
    minimum_insertable_extra_length_um: float
    final_planning_mode: str = "none"
    profile: MeanderPlanningProfile | None = None

    def to_info_dict(self) -> dict[str, object]:
        """The report as the GDS meta info dict (``meander_insertion_report``).

        Lengths are floats in um. ``realized_extra_by_edge`` is written as the
        list ``realized_extra_length_um_by_edge`` of ``edge`` and
        ``extra_length_um`` entries. When a profile was built, its entries
        are merged into the top level of the dict.
        """
        info: dict[str, object] = {
            "results": [entry.to_info_dict() for entry in self.results],
            "total_requested_extra_length_um": float(self.total_requested_extra_length_um),
            "total_inserted_extra_length_um": float(self.total_inserted_extra_length_um),
            "total_physical_inserted_extra_length_um": float(
                self.total_physical_inserted_extra_length_um
            ),
            # Bookkeeping by physical edge (2026-10-05, owner): the meander a
            # result books on its requiring edge may be drawn on a
            # delay-insertion candidate upstream (through heaters, or on all
            # inputs of an MMI whose outputs all require the same extra), so
            # a net's realized length is its routed length plus THIS entry,
            # not plus the result's inserted_extra_length_um.
            "realized_extra_length_um_by_edge": [
                {"edge": edge_key_to_dict(edge_key), "extra_length_um": float(extra)}
                for edge_key, extra in self.realized_extra_by_edge.items()
            ],
            "total_disregarded_extra_length_um": float(self.total_disregarded_extra_length_um),
            "unmatched_length_um": float(self.unmatched_length_um),
            "minimum_insertable_extra_length_um": float(self.minimum_insertable_extra_length_um),
            "final_planning_mode": self.final_planning_mode,
        }
        if self.profile is not None:
            info.update(self.profile.to_info_dict())
        return info


@dataclass(frozen=True)
class RouteRustPipelineResult:
    """What ``route_match_and_realize`` returns: the routed layout and its records.

    ``debug_artifacts`` is the routing session's record.
    ``path_length_analysis_info`` and ``meander_requirements_info`` are the
    path-length analysis and its meander requirements in the form written
    to the GDS meta info. ``pipeline_timings_s`` maps a pipeline step's name
    to its wall time in seconds.
    """

    routed_layout: Component
    debug_artifacts: RustRouteDebugArtifacts
    path_length_analysis_info: dict[str, object] | None = None
    meander_requirements_info: list[dict[str, object]] | None = None
    #: the GDS meta info form of ``meander_insertion_report``
    meander_insertion_report_info: dict[str, object] | None = None
    pipeline_timings_s: dict[str, float] = field(default_factory=dict)
    meander_insertion_report: MeanderInsertionReport | None = None
