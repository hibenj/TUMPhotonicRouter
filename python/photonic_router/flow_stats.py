"""Legacy statistics collection for routing_flow."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from photonic_router.records.route_rust_types import (
        RouteRustPipelineResult,
        RouteSearchSummary,
        RustRouteDebugArtifacts,
    )


@dataclass
class RoutingFlowStats:
    """The timings and counters of one flow run, filled stage by stage; the SUMMARY line reads it."""

    benchmark_name: str | None = None
    total_time_s: float = 0.0
    instance_count: int = 0
    net_count: int = 0
    static_grid_width: int | None = None
    static_grid_height: int | None = None
    raw_blocked_cells: int | None = None
    blocked_cells: int | None = None
    port_open_cells: int = 0
    astar_time_s: float = 0.0
    route_attempts: int = 0
    #: None until per-route collection ran (``collect_route_stats`` or
    #: ``debug_timing``); the SUMMARY line prints ``-`` then, never a guessed 0.
    failed_attempts: int | None = None
    #: nets the detour pass re-routed; None when the pass did not run (prints ``-``)
    detour_pass_reroutes: int | None = None
    simple_route_count: int = 0
    repair_count: int = 0
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
    footprint_checks: int = 0
    footprint_rect_checks: int = 0
    crossing_candidate_checks: int = 0
    crossing_accepted: int = 0
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
    full_grid_fallbacks: int = 0
    search_loop_time_s: float = 0.0
    obstacle_map_prepare_time_s: float = 0.0
    simple_route_time_s: float = 0.0
    commit_prepare_time_s: float = 0.0
    commit_time_s: float = 0.0
    neighbor_generation_time_s: float = 0.0
    heap_operation_time_s: float = 0.0
    legality_check_time_s: float = 0.0
    reconstruction_time_s: float = 0.0
    #: None until the electrical step ran; a run that found no heater
    #: terminals records 0, not None, so the SUMMARY line can tell the two apart.
    electrical_terminal_groups: int | None = None
    electrical_pad_assignments: int | None = None
    electrical_detailed_routes: int | None = None
    electrical_failed_detailed_routes: int | None = None
    electrical_verification_success: bool | None = None
    verification_error_count: int | None = None
    verification_warning_count: int | None = None
    path_length_groups: int | None = None
    path_length_groups_over_tolerance: int | None = None
    meander_inserted_um: float | None = None
    route_attempt_records: list[dict[str, object]] = field(default_factory=list)
    step_times_s: dict[str, float] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        """Return every field under its own name as a plain dict.

        Values are passed through unchanged, so a field that is None until its
        step ran stays None. route_attempt_records and step_times_s are
        shallow copies.
        """
        return {
            "benchmark_name": self.benchmark_name,
            "total_time_s": self.total_time_s,
            "instance_count": self.instance_count,
            "net_count": self.net_count,
            "static_grid_width": self.static_grid_width,
            "static_grid_height": self.static_grid_height,
            "raw_blocked_cells": self.raw_blocked_cells,
            "blocked_cells": self.blocked_cells,
            "port_open_cells": self.port_open_cells,
            "astar_time_s": self.astar_time_s,
            "route_attempts": self.route_attempts,
            "failed_attempts": self.failed_attempts,
            "detour_pass_reroutes": self.detour_pass_reroutes,
            "simple_route_count": self.simple_route_count,
            "repair_count": self.repair_count,
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
            "footprint_checks": self.footprint_checks,
            "footprint_rect_checks": self.footprint_rect_checks,
            "crossing_candidate_checks": self.crossing_candidate_checks,
            "crossing_accepted": self.crossing_accepted,
            "crossing_reject_non_straight": self.crossing_reject_non_straight,
            "crossing_reject_not_perpendicular": self.crossing_reject_not_perpendicular,
            "crossing_reject_margin": self.crossing_reject_margin,
            "crossing_reject_wrong_order": self.crossing_reject_wrong_order,
            "crossing_reject_unexpected_owner": self.crossing_reject_unexpected_owner,
            "crossing_reject_unmatched_owner": self.crossing_reject_unmatched_owner,
            "crossing_reject_unmatched_centerline": (self.crossing_reject_unmatched_centerline),
            "crossing_reject_unmatched_footprint": (self.crossing_reject_unmatched_footprint),
            "crossing_reject_unmatched_route_centerline": (
                self.crossing_reject_unmatched_route_centerline
            ),
            "crossing_reject_unmatched_route_footprint": (
                self.crossing_reject_unmatched_route_footprint
            ),
            "crossing_reject_pending_straight": self.crossing_reject_pending_straight,
            "full_grid_fallbacks": self.full_grid_fallbacks,
            "search_loop_time_s": self.search_loop_time_s,
            "obstacle_map_prepare_time_s": self.obstacle_map_prepare_time_s,
            "simple_route_time_s": self.simple_route_time_s,
            "commit_prepare_time_s": self.commit_prepare_time_s,
            "commit_time_s": self.commit_time_s,
            "neighbor_generation_time_s": self.neighbor_generation_time_s,
            "heap_operation_time_s": self.heap_operation_time_s,
            "legality_check_time_s": self.legality_check_time_s,
            "reconstruction_time_s": self.reconstruction_time_s,
            "electrical_terminal_groups": self.electrical_terminal_groups,
            "electrical_pad_assignments": self.electrical_pad_assignments,
            "electrical_detailed_routes": self.electrical_detailed_routes,
            "electrical_failed_detailed_routes": self.electrical_failed_detailed_routes,
            "electrical_verification_success": self.electrical_verification_success,
            "verification_error_count": self.verification_error_count,
            "verification_warning_count": self.verification_warning_count,
            "path_length_groups": self.path_length_groups,
            "path_length_groups_over_tolerance": self.path_length_groups_over_tolerance,
            "meander_inserted_um": self.meander_inserted_um,
            "route_attempt_records": list(self.route_attempt_records),
            "step_times_s": dict(self.step_times_s),
        }


def record_initial_route_stats(stats: RoutingFlowStats | None) -> None:
    """Set the route timing fields to 0 before the optical router runs."""
    if stats is None:
        return
    stats.step_times_s["build_static_obstacle_map"] = 0.0
    stats.step_times_s["baseline_gdsfactory_routing"] = 0.0


#: Counters `populate_route_stats` copies from the route search summary to
#: the field of the same name.
_SUMMARY_COUNTERS = (
    "simple_route_count",
    "repair_count",
    "expanded_states",
    "generated_neighbors",
    "heap_pushes",
    "heap_pops",
    "skipped_duplicate_heap_entries",
    "stale_generation_heap_entries",
    "closed_heap_entries",
    "max_heap_size",
    "dense_search_states",
    "dense_search_storage_bytes",
    "best_cost_updates",
    "parent_updates",
    "obstacle_clearance_checks",
    "footprint_checks",
    "footprint_rect_checks",
    "crossing_candidate_checks",
    "crossing_accepted",
    "crossing_reject_non_straight",
    "crossing_reject_not_perpendicular",
    "crossing_reject_margin",
    "crossing_reject_wrong_order",
    "crossing_reject_unexpected_owner",
    "crossing_reject_unmatched_owner",
    "crossing_reject_unmatched_centerline",
    "crossing_reject_unmatched_footprint",
    "crossing_reject_unmatched_route_centerline",
    "crossing_reject_unmatched_route_footprint",
    "crossing_reject_pending_straight",
    "full_grid_fallbacks",
)

#: Times it copies likewise: `<name>_us` of the summary (microseconds) to
#: `<name>_s` (seconds).
_SUMMARY_TIMES = (
    "neighbor_generation_time",
    "heap_operation_time",
    "legality_check_time",
    "reconstruction_time",
    "search_loop_time",
    "obstacle_map_prepare_time",
    "simple_route_time",
    "commit_prepare_time",
    "commit_time",
)


def populate_route_stats(
    stats: RoutingFlowStats,
    *,
    route_result: RouteRustPipelineResult,
    debug_artifacts: RustRouteDebugArtifacts,
    route_summary: RouteSearchSummary,
    route_attempt_records: list[dict[str, object]],
    route_time_s: float,
) -> None:
    """Copy the optical route counters and timings into `stats`."""
    stats.step_times_s["baseline_gdsfactory_routing"] = route_time_s
    if "build_static_obstacle_map" not in stats.step_times_s:
        stats.step_times_s["build_static_obstacle_map"] = 0.0
    for name, elapsed_s in route_result.pipeline_timings_s.items():
        stats.step_times_s[str(name)] = float(elapsed_s)
    if debug_artifacts.realization_grid_spec is not None:
        width, height, *_ = debug_artifacts.realization_grid_spec
        stats.static_grid_width = int(width)
        stats.static_grid_height = int(height)
    blocked_count = len(debug_artifacts.static_blocked_cells)
    if blocked_count == 0:
        blocked_count = int(debug_artifacts.static_obstacle_count or 0)
    if blocked_count > 0:
        stats.blocked_cells = blocked_count
        stats.raw_blocked_cells = blocked_count
        port_open_count = int(debug_artifacts.static_port_open_count or 0)
        stats.port_open_cells = port_open_count
    stats.astar_time_s = float(route_summary.astar_elapsed_s)
    stats.route_attempts = int(route_summary.route_attempts)
    # The failure count comes from the timing buckets, which only exist when
    # per-route collection ran (``collect_route_stats`` or ``debug_timing``);
    # without it the buckets are empty and the count would be a guessed 0.
    stats.failed_attempts = (
        int(route_summary.failed_attempts) if int(route_summary.route_attempts) > 0 else None
    )
    stats.detour_pass_reroutes = debug_artifacts.detour_pass_reroutes
    for name in _SUMMARY_COUNTERS:
        setattr(stats, name, int(getattr(route_summary, name)))
    for name in _SUMMARY_TIMES:
        setattr(stats, f"{name}_s", float(getattr(route_summary, f"{name}_us")) / 1_000_000.0)
    stats.route_attempt_records = route_attempt_records


def format_flow_summary_line(stats: RoutingFlowStats | None) -> str:
    """One fixed, grep-able line per flow run (`SUMMARY key=value ...`).

    Every value a reader or an agent needs to judge a run without parsing the
    stage output: nets and route failures, the photonic verifier's counts,
    path-length matching, the inserted meander length, the electrical step's
    counts and verdict, and the wall time. A field the run did not produce
    prints as `-` (a stage skipped), never as a guessed zero.
    """

    if stats is None:
        return "SUMMARY stats=off"

    def cell(value: object, digits: int | None = None) -> str:
        if value is None:
            return "-"
        if isinstance(value, bool):
            return "yes" if value else "no"
        if isinstance(value, float) and digits is not None:
            return f"{value:.{digits}f}"
        return str(value)

    fields = (
        ("benchmark", cell(stats.benchmark_name)),
        ("nets", cell(stats.net_count)),
        ("failed_attempts", cell(stats.failed_attempts)),
        ("detours", cell(stats.detour_pass_reroutes)),
        ("verifier_errors", cell(stats.verification_error_count)),
        ("verifier_warnings", cell(stats.verification_warning_count)),
        ("plm_groups", cell(stats.path_length_groups)),
        ("plm_over_tolerance", cell(stats.path_length_groups_over_tolerance)),
        ("meander_inserted_um", cell(stats.meander_inserted_um, 1)),
        ("electrical_heaters", cell(stats.electrical_terminal_groups)),
        ("electrical_pads", cell(stats.electrical_pad_assignments)),
        ("electrical_failed_wires", cell(stats.electrical_failed_detailed_routes)),
        ("electrical_ok", cell(stats.electrical_verification_success)),
        ("total_s", cell(stats.total_time_s, 1)),
    )
    return "SUMMARY " + " ".join(f"{key}={value}" for key, value in fields)


@dataclass(frozen=True)
class FlowRun:
    """What the stages of one flow run report with."""

    benchmark_name: str
    #: how many numbered steps the run prints (3, or 4 with electrical routing)
    total_steps: int
    #: the run's counters and timings; None when the caller collects none
    stats: RoutingFlowStats | None
    #: whether a stage prints how long it took
    debug_timing: bool = False
