"""Legacy statistics collection for routing_flow."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class RoutingFlowStats:
    """Compatibility container for legacy routing-flow timing/stat collection."""

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
    route_failures: int = 0
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
    electrical_terminal_groups: int = 0
    electrical_pad_assignments: int = 0
    electrical_detailed_routes: int = 0
    electrical_failed_detailed_routes: int = 0
    route_attempt_records: list[dict[str, object]] = field(default_factory=list)
    step_times_s: dict[str, float] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
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
            "route_failures": self.route_failures,
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
            "route_attempt_records": list(self.route_attempt_records),
            "step_times_s": dict(self.step_times_s),
        }


def record_initial_route_stats(stats: RoutingFlowStats | None) -> None:
    """Initialize legacy route timing fields before the optical router runs."""
    if stats is None:
        return
    stats.step_times_s["build_static_obstacle_map"] = 0.0
    stats.step_times_s["baseline_gdsfactory_routing"] = 0.0


def populate_route_stats(
    stats: RoutingFlowStats,
    *,
    route_result: object,
    debug_artifacts: object,
    route_summary: object,
    route_attempt_records: list[dict[str, object]],
    route_time_s: float,
) -> None:
    """Copy optical route counters into the legacy stats object."""
    stats.step_times_s["baseline_gdsfactory_routing"] = route_time_s
    if "build_static_obstacle_map" not in stats.step_times_s:
        stats.step_times_s["build_static_obstacle_map"] = 0.0
    for name, elapsed_s in getattr(route_result, "pipeline_timings_s", {}).items():
        stats.step_times_s[str(name)] = float(elapsed_s)
    if getattr(debug_artifacts, "realization_grid_spec", None) is not None:
        width, height, *_ = debug_artifacts.realization_grid_spec
        stats.static_grid_width = int(width)
        stats.static_grid_height = int(height)
    blocked_count = len(getattr(debug_artifacts, "static_blocked_cells", ()))
    if blocked_count == 0:
        blocked_count = int(getattr(debug_artifacts, "static_obstacle_count", 0) or 0)
    if blocked_count > 0:
        stats.blocked_cells = blocked_count
        stats.raw_blocked_cells = blocked_count
        port_open_count = int(getattr(debug_artifacts, "static_port_open_count", 0) or 0)
        stats.port_open_cells = port_open_count
    stats.astar_time_s = float(route_summary.astar_elapsed_s)
    stats.route_attempts = int(route_summary.route_attempts)
    stats.route_failures = int(route_summary.route_failures)
    stats.simple_route_count = int(route_summary.simple_route_count)
    stats.repair_count = int(route_summary.repair_count)
    stats.expanded_states = int(route_summary.expanded_states)
    stats.generated_neighbors = int(route_summary.generated_neighbors)
    stats.heap_pushes = int(route_summary.heap_pushes)
    stats.heap_pops = int(route_summary.heap_pops)
    stats.skipped_duplicate_heap_entries = int(route_summary.skipped_duplicate_heap_entries)
    stats.stale_generation_heap_entries = int(route_summary.stale_generation_heap_entries)
    stats.closed_heap_entries = int(route_summary.closed_heap_entries)
    stats.max_heap_size = int(route_summary.max_heap_size)
    stats.dense_search_states = int(route_summary.dense_search_states)
    stats.dense_search_storage_bytes = int(route_summary.dense_search_storage_bytes)
    stats.best_cost_updates = int(route_summary.best_cost_updates)
    stats.parent_updates = int(route_summary.parent_updates)
    stats.obstacle_clearance_checks = int(route_summary.obstacle_clearance_checks)
    stats.footprint_checks = int(route_summary.footprint_checks)
    stats.footprint_rect_checks = int(route_summary.footprint_rect_checks)
    stats.crossing_candidate_checks = int(route_summary.crossing_candidate_checks)
    stats.crossing_accepted = int(route_summary.crossing_accepted)
    stats.crossing_reject_non_straight = int(route_summary.crossing_reject_non_straight)
    stats.crossing_reject_not_perpendicular = int(route_summary.crossing_reject_not_perpendicular)
    stats.crossing_reject_margin = int(route_summary.crossing_reject_margin)
    stats.crossing_reject_wrong_order = int(route_summary.crossing_reject_wrong_order)
    stats.crossing_reject_unexpected_owner = int(route_summary.crossing_reject_unexpected_owner)
    stats.crossing_reject_unmatched_owner = int(route_summary.crossing_reject_unmatched_owner)
    stats.crossing_reject_unmatched_centerline = int(
        route_summary.crossing_reject_unmatched_centerline
    )
    stats.crossing_reject_unmatched_footprint = int(
        route_summary.crossing_reject_unmatched_footprint
    )
    stats.crossing_reject_unmatched_route_centerline = int(
        route_summary.crossing_reject_unmatched_route_centerline
    )
    stats.crossing_reject_unmatched_route_footprint = int(
        route_summary.crossing_reject_unmatched_route_footprint
    )
    stats.crossing_reject_pending_straight = int(route_summary.crossing_reject_pending_straight)
    stats.full_grid_fallbacks = int(route_summary.full_grid_fallbacks)
    stats.neighbor_generation_time_s = (
        float(route_summary.neighbor_generation_time_us) / 1_000_000.0
    )
    stats.heap_operation_time_s = float(route_summary.heap_operation_time_us) / 1_000_000.0
    stats.legality_check_time_s = float(route_summary.legality_check_time_us) / 1_000_000.0
    stats.reconstruction_time_s = float(route_summary.reconstruction_time_us) / 1_000_000.0
    stats.search_loop_time_s = float(route_summary.search_loop_time_us) / 1_000_000.0
    stats.obstacle_map_prepare_time_s = (
        float(route_summary.obstacle_map_prepare_time_us) / 1_000_000.0
    )
    stats.simple_route_time_s = float(route_summary.simple_route_time_us) / 1_000_000.0
    stats.commit_prepare_time_s = float(route_summary.commit_prepare_time_us) / 1_000_000.0
    stats.commit_time_s = float(route_summary.commit_time_us) / 1_000_000.0
    stats.route_attempt_records = route_attempt_records
