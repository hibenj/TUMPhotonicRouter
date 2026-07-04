"""Photonic routing flow orchestrator.

This module orchestrates the photonic routing flow:
1. Load benchmark (schematic)
2. Translate schematic to unrouted layout
3. Route connections using the Rust router backend
4. Optionally route heater electrical metal
5. Generate final routed layout
"""

import argparse
from collections import Counter
from dataclasses import dataclass, field
import importlib
import time
from pathlib import Path
import webbrowser
from typing import Any, cast

from benchmark_metadata import load_benchmark_metadata
from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from translation.electrical import (
    DEFAULT_BUS_WIDTH_UM,
    DEFAULT_PAD_PITCH_UM,
    DEFAULT_WIRE_WIDTH_UM,
    ElectricalRoutingConfig,
    ElectricalVerificationIssue,
    ElectricalVerificationResult,
    ElectricalRoutingResult,
    route_electrical_heaters,
)
from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import (
    DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    RipupRerouteConfig,
    route_match_and_realize,
)
from translation.route_rust_types import DEFAULT_MEANDER_MAX_HEIGHT_UM
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig

DebugSvgSelector = bool | int | str | range | set[int] | list[int] | tuple[int, ...]

# Edit these values when running `routing_flow.py` directly from an IDE or file.
# Command-line arguments override these defaults.
SCRIPT_BENCHMARK = "benes_16x16"
SCRIPT_DEBUG_SVGS: DebugSvgSelector = False # Examples: True, "all", "5-10", "2,5-10"
SCRIPT_DEBUG_TIMING = True
SCRIPT_DEBUG_MEANDERS = False
SCRIPT_VERBOSE_ROUTES = False
SCRIPT_SHOW_KLAYOUT = False
SCRIPT_ALLOW_45_DEGREE_TURNS = True
SCRIPT_BEND_RADIUS_UM = 5.0
SCRIPT_ENABLE_PATH_LENGTH_MATCHING = False
SCRIPT_PATH_LENGTH_MATCH_OUTPUTS = False
SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM = DEFAULT_MEANDER_MAX_HEIGHT_UM
SCRIPT_ENABLE_CROSSINGS = True
SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING
SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS = 6
SCRIPT_PROACTIVE_CONGESTION_WEIGHT = 0.0
SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS = 0
SCRIPT_MAX_ITERATIONS = 5_000_000
SCRIPT_ROUTING_WINDOW_SCALE = 0.05
SCRIPT_INCLUDE_HEATER_OBSTACLES = True
SCRIPT_OBSTACLE_MODE = "bounding_boxes"
SCRIPT_GRID_SIZE_UM = 2.0
SCRIPT_WAVEGUIDE_CLEARANCE_UM = 0.0
SCRIPT_HEATER_CLEARANCE_UM = 10.0
SCRIPT_CHIP_ADD_X_UM = 0.0
SCRIPT_CROSSING_MODE = "lidar-pure"
SCRIPT_CHIP_ADD_Y_UM = 40.0
SCRIPT_OBSTACLE_CLEARANCE_UM = SCRIPT_WAVEGUIDE_CLEARANCE_UM
SCRIPT_CLEAR_PORT_OPEN_CELLS_FROM_STATIC = False
SCRIPT_ENABLE_RIPUP_REROUTE = True
SCRIPT_RIPUP_MAX_ROUNDS = 4
SCRIPT_RIPUP_MAX_VICTIMS = 8
SCRIPT_RIPUP_HISTORY_WEIGHT = 2.0
SCRIPT_RIPUP_HISTORY_INCREMENT = 1
SCRIPT_ATTEMPT_DIAGNOSTICS = False
SCRIPT_ENABLE_ELECTRICAL_ROUTING = False
SCRIPT_ELECTRICAL_PAD_SIDE = "top"
SCRIPT_ELECTRICAL_GRID_PITCH_UM = 10.0
SCRIPT_ELECTRICAL_OBSTACLE_CLEARANCE_UM = 10.0
SCRIPT_ELECTRICAL_WIRE_WIDTH_UM = DEFAULT_WIRE_WIDTH_UM
SCRIPT_ELECTRICAL_BUS_WIDTH_UM = DEFAULT_BUS_WIDTH_UM
SCRIPT_ELECTRICAL_TERMINAL_CONTACT_WIDTH_UM = 10.0
SCRIPT_ELECTRICAL_PAD_PITCH_UM = DEFAULT_PAD_PITCH_UM


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
            "crossing_reject_unmatched_centerline": (
                self.crossing_reject_unmatched_centerline
            ),
            "crossing_reject_unmatched_footprint": (
                self.crossing_reject_unmatched_footprint
            ),
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


def _parse_debug_svg_selector(debug_svgs: DebugSvgSelector) -> tuple[bool, set[int] | None]:
    """Parse debug SVG selection.

    Returns:
        `(enabled, selected_route_indices)`, where `selected_route_indices=None`
        means all route SVGs. Route indices are 1-based in netlist order.
    """
    if isinstance(debug_svgs, bool):
        return debug_svgs, None
    if isinstance(debug_svgs, int):
        if debug_svgs < 1:
            raise ValueError("debug_svgs integer selectors must be >= 1")
        return True, {debug_svgs}
    if isinstance(debug_svgs, range):
        indices = set(debug_svgs)
        if any(index < 1 for index in indices):
            raise ValueError("debug_svgs range selectors must contain only indices >= 1")
        return True, indices
    if isinstance(debug_svgs, (set, list, tuple)):
        indices = {int(index) for index in debug_svgs}
        if any(index < 1 for index in indices):
            raise ValueError("debug_svgs sequence selectors must contain only indices >= 1")
        return bool(indices), indices
    if isinstance(debug_svgs, str):
        selector = debug_svgs.strip().lower()
        if selector in {"", "false", "off", "none", "no"}:
            return False, None
        if selector in {"true", "on", "yes", "all", "*"}:
            return True, None

        indices: set[int] = set()
        for part in selector.split(","):
            token = part.strip()
            if not token:
                continue
            if "-" in token:
                start_text, end_text = token.split("-", 1)
                start = int(start_text.strip())
                end = int(end_text.strip())
                if start < 1 or end < 1:
                    raise ValueError("debug_svgs range selectors must use indices >= 1")
                if start > end:
                    raise ValueError(f"debug_svgs range start must be <= end: {token!r}")
                indices.update(range(start, end + 1))
            else:
                index = int(token)
                if index < 1:
                    raise ValueError("debug_svgs route selectors must be >= 1")
                indices.add(index)
        if not indices:
            return False, None
        return True, indices

    raise TypeError(
        "debug_svgs must be a bool, int, range, sequence of ints, or selector string"
    )


def _format_debug_route_indices(indices: set[int]) -> str:
    if not indices:
        return "<none>"

    ranges: list[str] = []
    sorted_indices = sorted(indices)
    start = sorted_indices[0]
    previous = start
    for index in sorted_indices[1:]:
        if index == previous + 1:
            previous = index
            continue
        ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
        start = index
        previous = index
    ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
    return ",".join(ranges)


def _route_attempt_as_dict(record: object) -> dict[str, object]:
    as_dict = getattr(record, "as_dict", None)
    if callable(as_dict):
        result = as_dict()
        if isinstance(result, dict):
            return dict(result)
    if isinstance(record, dict):
        return dict(record)
    return {}


def _route_attempt_float(record: dict[str, object], key: str) -> float:
    value = record.get(key, 0.0)
    try:
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return float(value)
    except (TypeError, ValueError):
        pass
    return 0.0


def _route_attempt_int(record: dict[str, object], key: str) -> int:
    value = record.get(key, 0)
    try:
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return int(value)
    except (TypeError, ValueError):
        pass
    return 0


def _format_slowest_route_attempt_lines(
    records: list[dict[str, object]],
    *,
    limit: int = 8,
) -> list[str]:
    timed_records = [
        record
        for record in records
        if _route_attempt_float(record, "elapsed_s") > 0.0
    ]
    if not timed_records:
        return []

    slowest = sorted(
        timed_records,
        key=lambda record: _route_attempt_float(record, "elapsed_s"),
        reverse=True,
    )[: max(1, int(limit))]
    lines: list[str] = []
    for record in slowest:
        elapsed_s = _route_attempt_float(record, "elapsed_s")
        route_index = _route_attempt_int(record, "route_index")
        attempt_index = _route_attempt_int(record, "attempt_index")
        expanded_states = _route_attempt_int(record, "expanded_states")
        generated_neighbors = _route_attempt_int(record, "generated_neighbors")
        window_attempts = _route_attempt_int(record, "window_attempts")
        dense_grid_cells = _route_attempt_int(record, "dense_grid_cells")
        search_loop_time_s = _route_attempt_float(record, "search_loop_time_s")
        simple_route_time_s = _route_attempt_float(record, "simple_route_time_s")
        commit_time_s = _route_attempt_float(record, "commit_time_s")
        full_grid = bool(record.get("used_full_grid_fallback", False))
        status = "failed" if bool(record.get("failed", False)) else "ok"
        route_kind = (
            "simple" if bool(record.get("used_simple_route", False)) else "astar"
        )
        parts = [
            f"#{attempt_index}",
            f"route[{route_index}]",
            str(record.get("net_name", "<unknown>")),
            str(record.get("bucket_name", "<unknown>")),
            f"{elapsed_s:.4f}s",
            status,
            route_kind,
            f"expanded={expanded_states}",
            f"generated={generated_neighbors}",
            f"windows={window_attempts}",
        ]
        if dense_grid_cells:
            parts.append(f"dense_cells={dense_grid_cells}")
        if search_loop_time_s > 0.0:
            parts.append(f"search_loop={search_loop_time_s:.4f}s")
        if simple_route_time_s > 0.0:
            parts.append(f"simple_probe={simple_route_time_s:.4f}s")
        if commit_time_s > 0.0:
            parts.append(f"commit={commit_time_s:.4f}s")
        if full_grid:
            parts.append("full_grid")
        lines.append("            - " + ", ".join(parts))
    return lines


def _format_slowest_route_net_lines(
    records: list[dict[str, object]],
    *,
    limit: int = 8,
) -> list[str]:
    grouped: dict[tuple[int, str], dict[str, object]] = {}
    for record in records:
        elapsed_s = _route_attempt_float(record, "elapsed_s")
        if elapsed_s <= 0.0:
            continue
        route_index = _route_attempt_int(record, "route_index")
        net_name = str(record.get("net_name", "<unknown>"))
        key = (route_index, net_name)
        group = grouped.setdefault(
            key,
            {
                "route_index": route_index,
                "net_name": net_name,
                "elapsed_s": 0.0,
                "attempts": 0,
                "failures": 0,
                "expanded_states": 0,
                "generated_neighbors": 0,
                "buckets": set(),
            },
        )
        group["elapsed_s"] = _route_attempt_float(group, "elapsed_s") + elapsed_s
        group["attempts"] = _route_attempt_int(group, "attempts") + 1
        group["failures"] = _route_attempt_int(group, "failures") + int(
            bool(record.get("failed", False))
        )
        group["expanded_states"] = _route_attempt_int(
            group,
            "expanded_states",
        ) + _route_attempt_int(
            record,
            "expanded_states",
        )
        group["generated_neighbors"] = _route_attempt_int(
            group,
            "generated_neighbors",
        ) + _route_attempt_int(
            record,
            "generated_neighbors",
        )
        buckets = group["buckets"]
        if isinstance(buckets, set):
            buckets.add(str(record.get("bucket_name", "<unknown>")))

    if not grouped:
        return []

    slowest = sorted(
        grouped.values(),
        key=lambda group: _route_attempt_float(group, "elapsed_s"),
        reverse=True,
    )[: max(1, int(limit))]
    lines: list[str] = []
    for group in slowest:
        buckets = group.get("buckets", set())
        bucket_text = (
            "/".join(sorted(buckets))
            if isinstance(buckets, set) and buckets
            else "<unknown>"
        )
        parts = [
            f"route[{_route_attempt_int(group, 'route_index')}]",
            str(group["net_name"]),
            f"{_route_attempt_float(group, 'elapsed_s'):.4f}s",
            f"attempts={_route_attempt_int(group, 'attempts')}",
            f"failures={_route_attempt_int(group, 'failures')}",
            f"expanded={_route_attempt_int(group, 'expanded_states')}",
            f"generated={_route_attempt_int(group, 'generated_neighbors')}",
            f"buckets={bucket_text}",
        ]
        lines.append("            - " + ", ".join(parts))
    return lines


def _parse_bool_flag(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"expected boolean value, got {value!r}")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the photonic routing flow for a benchmark."
    )
    parser.add_argument(
        "benchmark",
        nargs="?",
        default=SCRIPT_BENCHMARK,
        help=f"Benchmark module name from benchmarks/ (default: {SCRIPT_BENCHMARK}).",
    )
    parser.add_argument(
        "--debug-svgs",
        nargs="?",
        const="all",
        default=SCRIPT_DEBUG_SVGS,
        metavar="SELECTOR",
        help=(
            "Generate debug SVGs. Use without a value or with 'all' for every "
            "route, or pass 1-based route selectors like '5', '5-10', "
            "or '2,5-10'."
        ),
    )
    parser.add_argument(
        "--debug-timing",
        type=_parse_bool_flag,
        default=SCRIPT_DEBUG_TIMING,
        metavar="BOOL",
        help=f"Print timing details (default: {str(SCRIPT_DEBUG_TIMING).lower()}).",
    )
    parser.add_argument(
        "--debug-stop-after-route",
        type=int,
        default=None,
        metavar="N",
        help=(
            "Debug helper: build full-netlist obstacle/crossing context, but "
            "route only through 1-based route index N."
        ),
    )
    parser.add_argument(
        "--debug-meanders",
        action="store_true",
        default=SCRIPT_DEBUG_MEANDERS,
        help="Print verbose path-length and meander details.",
    )
    parser.add_argument(
        "--verbose-routes",
        action="store_true",
        default=SCRIPT_VERBOSE_ROUTES,
        help="Print per-net routing details, including simple vs A* route type.",
    )
    parser.add_argument(
        "--show-klayout",
        action="store_true",
        default=SCRIPT_SHOW_KLAYOUT,
        help="Open the routed layout in KLayout instead of only writing GDS.",
    )
    parser.add_argument(
        "--allow-45-degree-turns",
        type=_parse_bool_flag,
        default=SCRIPT_ALLOW_45_DEGREE_TURNS,
        metavar="BOOL",
        help=(
            "Allow 45-degree routing primitives "
            f"(default: {str(SCRIPT_ALLOW_45_DEGREE_TURNS).lower()})."
        ),
    )
    parser.add_argument(
        "--bend-radius-um",
        type=float,
        default=SCRIPT_BEND_RADIUS_UM,
        metavar="UM",
        help=(
            "Minimum optical waveguide bend radius in micrometers. The router "
            "rounds this up to an integer number of grid cells "
            f"(default: {SCRIPT_BEND_RADIUS_UM})."
        ),
    )
    parser.add_argument(
        "--path-length-matching",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_PATH_LENGTH_MATCHING,
        metavar="BOOL",
        help=(
            "Enable path-length matching analysis and realization "
            f"(default: {str(SCRIPT_ENABLE_PATH_LENGTH_MATCHING).lower()})."
        ),
    )
    parser.add_argument(
        "--path-length-match-outputs",
        type=_parse_bool_flag,
        default=SCRIPT_PATH_LENGTH_MATCH_OUTPUTS,
        metavar="BOOL",
        help=(
            "Also require all output nodes to have equal arrival delay when "
            "path-length matching is enabled "
            f"(default: {str(SCRIPT_PATH_LENGTH_MATCH_OUTPUTS).lower()})."
        ),
    )
    parser.add_argument(
        "--path-length-meander-height-um",
        type=float,
        default=SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
        metavar="UM",
        help=(
            "Maximum meander height used for path-length matching "
            f"(default: {SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM})."
        ),
    )
    parser.add_argument(
        "--crossings",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_CROSSINGS,
        metavar="BOOL",
        help=(
            "Build and pass topology-derived crossing constraints to the Rust "
            "router (default: "
            f"{str(SCRIPT_ENABLE_CROSSINGS).lower()})."
        ),
    )
    parser.add_argument(
        "--crossing-mode",
        choices=("window", "collision", "lidar-pure"),
        default=SCRIPT_CROSSING_MODE,
        help=(
            "Crossing search mode. 'window' preserves the existing expected-partner "
            "window search; 'collision' enables LiDAR-style collision-driven "
            "crossing legalization constrained by topology; 'lidar-pure' enables "
            "collision-driven crossing legalization without topology pair permissions "
            f"(default: {SCRIPT_CROSSING_MODE})."
        ),
    )
    parser.add_argument(
        "--min-straight-cells-per-crossing",
        type=int,
        default=SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
        metavar="N",
        help=(
            "Minimum straight access cells before and after each crossing "
            f"(default: {SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING})."
        ),
    )
    parser.add_argument(
        "--foreign-port-keepout-cells",
        type=int,
        default=SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
        metavar="N",
        help=(
            "Additional larger port keepout in grid cells. Unrelated nets are "
            "kept out; nets connected to the same instance can open it "
            f"(default: {SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS})."
        ),
    )
    parser.add_argument(
        "--proactive-congestion-weight",
        type=float,
        default=SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
        metavar="COST",
        help=(
            "Soft A* cost per nearby blocked cell beside straight moves "
            f"(default: {SCRIPT_PROACTIVE_CONGESTION_WEIGHT})."
        ),
    )
    parser.add_argument(
        "--proactive-congestion-radius-cells",
        type=int,
        default=SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
        metavar="N",
        help=(
            "Sideways grid radius used for proactive congestion counting "
            f"(default: {SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS})."
        ),
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=SCRIPT_MAX_ITERATIONS,
        metavar="N",
        help=f"Maximum A* state expansions per route attempt (default: {SCRIPT_MAX_ITERATIONS}).",
    )
    parser.add_argument(
        "--routing-window-scale",
        type=float,
        default=SCRIPT_ROUTING_WINDOW_SCALE,
        metavar="SCALE",
        help=(
            "Routing-window margin scale relative to source-target span "
            f"(default: {SCRIPT_ROUTING_WINDOW_SCALE})."
        ),
    )
    parser.add_argument(
        "--include-heater-obstacles",
        type=_parse_bool_flag,
        default=SCRIPT_INCLUDE_HEATER_OBSTACLES,
        metavar="BOOL",
        help=(
            "Include heater/metal layers as routing obstacles "
            f"(default: {str(SCRIPT_INCLUDE_HEATER_OBSTACLES).lower()})."
        ),
    )
    parser.add_argument(
        "--obstacle-mode",
        default=SCRIPT_OBSTACLE_MODE,
        help="Static obstacle mode passed to StaticObstacleMapConfig.",
    )
    parser.add_argument(
        "--grid-size-um",
        type=float,
        default=SCRIPT_GRID_SIZE_UM,
        metavar="UM",
        help=f"Optical routing grid resolution in micrometers (default: {SCRIPT_GRID_SIZE_UM}).",
    )
    parser.add_argument(
        "--waveguide-clearance-um",
        type=float,
        default=SCRIPT_WAVEGUIDE_CLEARANCE_UM,
        metavar="UM",
        help=(
            "Clearance in micrometers for existing optical/waveguide obstacles "
            f"(default: {SCRIPT_WAVEGUIDE_CLEARANCE_UM})."
        ),
    )
    parser.add_argument(
        "--obstacle-clearance-um",
        dest="waveguide_clearance_um",
        type=float,
        default=argparse.SUPPRESS,
        metavar="UM",
        help="Deprecated alias for --waveguide-clearance-um.",
    )
    parser.add_argument(
        "--heater-clearance-um",
        type=float,
        default=SCRIPT_HEATER_CLEARANCE_UM,
        metavar="UM",
        help=(
            "Clearance in micrometers for heater/metal obstacles when "
            "--include-heater-obstacles is true "
            f"(default: {SCRIPT_HEATER_CLEARANCE_UM})."
        ),
    )
    parser.add_argument(
        "--chip-add-x-um",
        type=float,
        default=SCRIPT_CHIP_ADD_X_UM,
        metavar="UM",
        help=(
            "Extra horizontal chip margin added to both left and right when "
            "the die bbox is computed automatically "
            f"(default: {SCRIPT_CHIP_ADD_X_UM})."
        ),
    )
    parser.add_argument(
        "--chip-add-y-um",
        type=float,
        default=SCRIPT_CHIP_ADD_Y_UM,
        metavar="UM",
        help=(
            "Extra vertical chip margin added to both bottom and top when "
            "the die bbox is computed automatically "
            f"(default: {SCRIPT_CHIP_ADD_Y_UM})."
        ),
    )
    parser.add_argument(
        "--clear-port-open-cells-from-static",
        type=_parse_bool_flag,
        default=SCRIPT_CLEAR_PORT_OPEN_CELLS_FROM_STATIC,
        metavar="BOOL",
        help=(
            "Clear port-open cells from static obstacles "
            f"(default: {str(SCRIPT_CLEAR_PORT_OPEN_CELLS_FROM_STATIC).lower()})."
        ),
    )
    parser.add_argument(
        "--ripup-reroute",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_RIPUP_REROUTE,
        metavar="BOOL",
        help=(
            "Enable conflict-probe rip-up and reroute "
            f"(default: {str(SCRIPT_ENABLE_RIPUP_REROUTE).lower()})."
        ),
    )
    parser.add_argument(
        "--ripup-max-rounds",
        type=int,
        default=SCRIPT_RIPUP_MAX_ROUNDS,
        metavar="N",
        help=f"Maximum repair rounds per failed net (default: {SCRIPT_RIPUP_MAX_ROUNDS}).",
    )
    parser.add_argument(
        "--ripup-max-victims",
        type=int,
        default=SCRIPT_RIPUP_MAX_VICTIMS,
        metavar="N",
        help=(
            "Maximum blocker routes to rip up per repair round "
            f"(default: {SCRIPT_RIPUP_MAX_VICTIMS})."
        ),
    )
    parser.add_argument(
        "--ripup-history-weight",
        type=float,
        default=SCRIPT_RIPUP_HISTORY_WEIGHT,
        metavar="W",
        help=f"A* history penalty weight during repair (default: {SCRIPT_RIPUP_HISTORY_WEIGHT}).",
    )
    parser.add_argument(
        "--ripup-history-increment",
        type=int,
        default=SCRIPT_RIPUP_HISTORY_INCREMENT,
        metavar="N",
        help=(
            "History penalty increment for probed/ripped route cells "
            f"(default: {SCRIPT_RIPUP_HISTORY_INCREMENT})."
        ),
    )
    parser.add_argument(
        "--attempt-diagnostics",
        action="store_true",
        default=SCRIPT_ATTEMPT_DIAGNOSTICS,
        help=(
            "Collect extra per-attempt window, obstacle-density, and rip-up "
            "diagnostics for slow or failed route attempts."
        ),
    )
    parser.add_argument(
        "--electrical-routing",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_ELECTRICAL_ROUTING,
        metavar="BOOL",
        help=(
            "Route heater electrical metal after optical routing "
            f"(default: {str(SCRIPT_ENABLE_ELECTRICAL_ROUTING).lower()})."
        ),
    )
    parser.add_argument(
        "--electrical-pad-side",
        choices=("top", "bottom"),
        default=SCRIPT_ELECTRICAL_PAD_SIDE,
        help=(
            "Side used for electrical bondpad placement "
            f"(default: {SCRIPT_ELECTRICAL_PAD_SIDE})."
        ),
    )
    parser.add_argument(
        "--electrical-grid-pitch-um",
        type=float,
        default=SCRIPT_ELECTRICAL_GRID_PITCH_UM,
        metavar="UM",
        help=(
            "Electrical routing grid pitch "
            f"(default: {SCRIPT_ELECTRICAL_GRID_PITCH_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-obstacle-clearance-um",
        type=float,
        default=SCRIPT_ELECTRICAL_OBSTACLE_CLEARANCE_UM,
        metavar="UM",
        help=(
            "Electrical routing obstacle clearance "
            f"(default: {SCRIPT_ELECTRICAL_OBSTACLE_CLEARANCE_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-wire-width-um",
        type=float,
        default=SCRIPT_ELECTRICAL_WIRE_WIDTH_UM,
        metavar="UM",
        help=(
            "Electrical individual route wire width "
            f"(default: {SCRIPT_ELECTRICAL_WIRE_WIDTH_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-bus-width-um",
        type=float,
        default=SCRIPT_ELECTRICAL_BUS_WIDTH_UM,
        metavar="UM",
        help=(
            "Electrical common bus route width "
            f"(default: {SCRIPT_ELECTRICAL_BUS_WIDTH_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-terminal-contact-width-um",
        type=float,
        default=SCRIPT_ELECTRICAL_TERMINAL_CONTACT_WIDTH_UM,
        metavar="UM",
        help=(
            "Minimum electrical terminal contact width "
            f"(default: {SCRIPT_ELECTRICAL_TERMINAL_CONTACT_WIDTH_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-pad-pitch-um",
        type=float,
        default=SCRIPT_ELECTRICAL_PAD_PITCH_UM,
        metavar="UM",
        help=(
            "Electrical bondpad pitch "
            f"(default: {SCRIPT_ELECTRICAL_PAD_PITCH_UM})."
        ),
    )
    parser.add_argument(
        "--enable-jps4",
        type=_parse_bool_flag,
        default=False,
        metavar="BOOL",
        help=(
            "Request the experimental Manhattan JPS4 accelerator when eligible "
            "(default: false). Pass 3A still falls back to baseline A*."
        ),
    )
    parser.add_argument(
        "--use-indexed-heap",
        type=_parse_bool_flag,
        default=False,
        metavar="BOOL",
        help=(
            "Use the experimental decrease-key indexed heap for dense A* "
            "instead of duplicate-entry BinaryHeap queueing (default: false)."
        ),
    )
    parser.add_argument(
        "--enable-simple-routes",
        type=_parse_bool_flag,
        default=True,
        metavar="BOOL",
        help=(
            "Enable straight/L/Z simple-route candidates before dense A* "
            "(default: true). Pass false for A*-only regression runs."
        ),
    )
    parser.add_argument(
        "--primitive-ordering",
        choices=("library", "long_straight_first", "target_biased"),
        default="library",
        help="Dense A* primitive iteration order experiment (default: library).",
    )
    parser.add_argument(
        "--heuristic-mode",
        choices=("distance", "heading_aware"),
        default="heading_aware",
        help="Dense A* heuristic mode (default: heading_aware).",
    )
    parser.add_argument(
        "--heap-tie-breaker",
        choices=("smaller_g", "larger_g"),
        default="smaller_g",
        help=(
            "Dense A* heap tie-breaker experiment. Default preserves the "
            "historical smaller-g behavior."
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> Component:
    args = _build_arg_parser().parse_args(argv)
    return run_routing_flow(
        args.benchmark,
        debug_svgs=args.debug_svgs,
        debug_timing=args.debug_timing,
        debug_stop_after_route_index=args.debug_stop_after_route,
        debug_meanders=args.debug_meanders,
        verbose_routes=args.verbose_routes,
        show_klayout=args.show_klayout,
        allow_45_degree_turns=args.allow_45_degree_turns,
        enable_jps4=args.enable_jps4,
        use_indexed_heap=args.use_indexed_heap,
        primitive_ordering=args.primitive_ordering,
        heuristic_mode=args.heuristic_mode,
        heap_tie_breaker=args.heap_tie_breaker,
        bend_radius_um=args.bend_radius_um,
        enable_path_length_matching=args.path_length_matching,
        path_length_match_outputs=args.path_length_match_outputs,
        path_length_meander_height_um=args.path_length_meander_height_um,
        enable_crossings=args.crossings,
        crossing_mode=args.crossing_mode,
        min_straight_cells_per_crossing=args.min_straight_cells_per_crossing,
        foreign_port_keepout_cells=args.foreign_port_keepout_cells,
        proactive_congestion_weight=args.proactive_congestion_weight,
        proactive_congestion_radius_cells=args.proactive_congestion_radius_cells,
        max_iterations=args.max_iterations,
        enable_simple_routes=args.enable_simple_routes,
        routing_window_scale=args.routing_window_scale,
        include_heater_obstacles=args.include_heater_obstacles,
        ripup_reroute_config=RipupRerouteConfig(
            enabled=args.ripup_reroute,
            max_rounds=args.ripup_max_rounds,
            max_victims_per_failure=args.ripup_max_victims,
            history_weight=args.ripup_history_weight,
            history_increment=args.ripup_history_increment,
        ),
        collect_attempt_diagnostics=args.attempt_diagnostics,
        enable_electrical_routing=args.electrical_routing,
        electrical_config=ElectricalRoutingConfig(
            pad_side=args.electrical_pad_side,
            routing_grid_pitch_um=args.electrical_grid_pitch_um,
            obstacle_clearance_um=args.electrical_obstacle_clearance_um,
            wire_width_um=args.electrical_wire_width_um,
            bus_width_um=args.electrical_bus_width_um,
            terminal_contact_width_um=args.electrical_terminal_contact_width_um,
            pad_pitch_um=args.electrical_pad_pitch_um,
        ),
        static_obstacle_config=StaticObstacleMapConfig(
            grid_size_um=args.grid_size_um,
            obstacle_mode=args.obstacle_mode,
            clearance_um=args.waveguide_clearance_um,
            heater_clearance_um=args.heater_clearance_um,
            chip_add_x_um=args.chip_add_x_um,
            chip_add_y_um=args.chip_add_y_um,
            clear_port_open_cells_from_static=args.clear_port_open_cells_from_static,
        ),
    )


def load_benchmark(benchmark_name: str) -> Schematic:
    """Load a benchmark schematic from the benchmarks directory.

    Parameters:
        benchmark_name: The name of the benchmark module (e.g., 'TOY').
                       The module must have a `build_schematic()` function.

    Returns:
        A gdsfactory Schematic object.

    Raises:
        ModuleNotFoundError: If the benchmark module is not found.
        AttributeError: If the benchmark doesn't have a `build_schematic()` function.
    """
    try:
        benchmark_module = importlib.import_module(f"benchmarks.{benchmark_name}")
        schematic = benchmark_module.build_schematic()

        if not isinstance(schematic, Schematic):
            raise TypeError(
                f"Expected Schematic from {benchmark_name}.build_schematic(), "
                f"got {type(schematic)}"
            )

        return schematic
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            f"Benchmark '{benchmark_name}' not found in benchmarks/ directory"
        ) from e
    except AttributeError as e:
        raise AttributeError(
            f"Benchmark '{benchmark_name}' must have a 'build_schematic()' function"
        ) from e


def _component_info(component: Component) -> Any:
    """Return a mutable component info object, creating one for test doubles."""

    info = getattr(component, "info", None)
    if info is None:
        info = {}
        setattr(component, "info", info)
    return info


def _copy_component_info(source: Component, target: Component) -> None:
    source_info = getattr(source, "info", None)
    if source_info is None:
        return
    target_info = _component_info(target)
    for key, value in getattr(source_info, "items", lambda: ())():
        if key not in target_info:
            target_info[key] = value


def _electrical_config_summary(
    config: ElectricalRoutingConfig | None,
) -> dict[str, Any]:
    if config is None:
        config = ElectricalRoutingConfig()
    keys = (
        "pad_side",
        "bus_side",
        "routing_grid_pitch_um",
        "obstacle_clearance_um",
        "wire_width_um",
        "bus_width_um",
        "terminal_contact_width_um",
        "pad_pitch_um",
        "bondpad_width_um",
        "common_bus_bondpad_width_um",
        "common_bus_bondpad_length_um",
        "bondpad_length_um",
        "pad_offset_um",
        "pad_access_depth_um",
        "common_bus_pad_position",
        "individual_route_spacing_um",
        "obstacle_mode",
        "clearance_metric",
        "metal_layer",
        "pad_marker_layer",
        "heater_layers",
        "metal_obstacle_layers",
    )
    summary: dict[str, Any] = {}
    for key in keys:
        value = getattr(config, key, None)
        if value is None:
            continue
        if isinstance(value, tuple):
            summary[key] = tuple(value)
            continue
        summary[key] = value
    return summary


def _electrical_summary(
    result: ElectricalRoutingResult,
    config: ElectricalRoutingConfig | None = None,
) -> dict[str, Any]:
    detailed_routes = result.detailed_bundle_routes
    failed_detailed_routes = (
        tuple(detailed_routes.failed_routes) if detailed_routes is not None else ()
    )
    verification = cast(
        ElectricalVerificationResult | None,
        getattr(result, "verification", None),
    )
    verification_issues: tuple[ElectricalVerificationIssue, ...] = (
        verification.issues if verification is not None else ()
    )
    issue_counts = Counter(issue.code for issue in verification_issues)
    realization_metrics = (
        dict(result.routed_component.info.get("electrical_metal_realization", {}))
        if result.routed_component is not None
        else {}
    )
    debug_artifacts = dict(result.debug_artifacts)
    return {
        "config": _electrical_config_summary(config),
        "terminal_group_count": len(result.terminal_groups),
        "common_bus_success": result.common_bus.success,
        "failed_heaters": tuple(result.common_bus.failed_heaters),
        "pad_assignment_count": (
            len(result.pad_plan.assignments) if result.pad_plan is not None else 0
        ),
        "common_bus_escape_success": (
            result.common_bus_escape.success
            if result.common_bus_escape is not None
            else None
        ),
        "detailed_route_count": (
            len(detailed_routes.routes) if detailed_routes is not None else 0
        ),
        "failed_detailed_route_count": len(failed_detailed_routes),
        "failed_detailed_routes": tuple(
            {
                "terminal_id": route.terminal.id,
                "reason": route.reason,
            }
            for route in failed_detailed_routes
        ),
        "verification_success": verification.success if verification is not None else None,
        "verification_error_count": verification.error_count if verification is not None else 0,
        "verification_warning_count": (
            verification.warning_count if verification is not None else 0
        ),
        "verification_issue_counts": dict(sorted(issue_counts.items())),
        "verification_metrics": (
            dict(verification.metrics)
            if verification is not None
            else {}
        ),
        "realization_metrics": realization_metrics,
        "verification_issues": (
            tuple(
                {
                    "code": issue.code,
                    "message": issue.message,
                    "severity": issue.severity,
                    "net_id": issue.net_id,
                    "details": dict(issue.details),
                }
                for issue in verification_issues
            )
            if verification is not None
            else ()
        ),
        "debug_artifacts": debug_artifacts,
        "debug_artifact_count": len(debug_artifacts),
    }


def _electrical_failure_summary(result: ElectricalRoutingResult) -> str:
    summary = _electrical_summary(result)
    return (
        "Electrical routing failed to produce a routed component: "
        f"failed_heaters={summary['failed_heaters']}, "
        f"failed_detailed_route_count={summary['failed_detailed_route_count']}"
    )


def run_routing_flow(
    benchmark_name: str,
    *,
    debug_svgs: DebugSvgSelector = False,
    show_unrouted: bool | None = None,
    show_routed: bool | None = None,
    show_debug_svgs: DebugSvgSelector | None = None,
    show_static_obstacles_svg: bool | None = None,
    debug_timing: bool = False,
    debug_stop_after_route_index: int | None = None,
    debug_meanders: bool = False,
    verbose_routes: bool = False,
    show_klayout: bool = False,
    enable_path_length_matching: bool = False,
    path_length_match_outputs: bool = False,
    path_length_meander_height_um: float = SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    enable_crossings: bool = False,
    crossing_mode: str = "window",
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
    proactive_congestion_weight: float = SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
    proactive_congestion_radius_cells: int = SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
    allow_45_degree_turns: bool = SCRIPT_ALLOW_45_DEGREE_TURNS,
    bend_radius_um: float = SCRIPT_BEND_RADIUS_UM,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    enable_simple_routes: bool = True,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    heap_tie_breaker: str = "smaller_g",
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    include_heater_obstacles: bool = False,
    grid_size_um: float = SCRIPT_GRID_SIZE_UM,
    waveguide_clearance_um: float | None = None,
    heater_clearance_um: float | None = None,
    obstacle_clearance_um: float | None = None,
    chip_add_x_um: float = SCRIPT_CHIP_ADD_X_UM,
    chip_add_y_um: float = SCRIPT_CHIP_ADD_Y_UM,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    static_obstacle_config: StaticObstacleMapConfig | None = None,
    enable_electrical_routing: bool = False,
    electrical_config: ElectricalRoutingConfig | None = None,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    stats: RoutingFlowStats | None = None,
) -> Component:
    """Execute the routing flow for a given benchmark.

    Parameters:
        benchmark_name: Name of the benchmark to run (e.g., 'TOY').
        debug_svgs: If True or "all", generate all debug SVGs into build/.
                    If a selector such as "5-10", "5", or "2,5-10" is
                    provided, generate only matching per-route SVGs by
                    1-based net order. Static obstacle SVGs are still
                    generated when debug SVGs are enabled.
        debug_timing: If True, print timing information for each stage.
        debug_meanders: If True, print verbose path-length and meander
                      insertion details when path-length matching is enabled.
        verbose_routes: If True, print per-net routing progress and whether
                      each route used the simple router or A*.
        show_klayout: If True, open the final routed layout in KLayout via
                      `Component.show()`.
        show_unrouted: Legacy alias. Currently unused (kept for compatibility).
        show_routed: Legacy alias for `show_klayout`.
        show_debug_svgs: Legacy alias for `debug_svgs`.
        show_static_obstacles_svg: Legacy alias for enabling debug SVG output.
        stats: Optional legacy stats collector. If provided, step metrics are
               populated in-place.
        enable_path_length_matching: If True, run post-route path-length
                      analysis and compute per-edge missing lengths.
        path_length_match_outputs: If True, add output-arrival equalization
                      requirements after local path-length matching.
        path_length_meander_height_um: Maximum meander height used when
                      inserting path-length matching meanders.
        crossing_half_size_cells: Crossing keepout half-size in grid cells.
                      The default 0 derives it from the crossing component bbox.
        crossing_mode: Crossing routing mode. "window" uses the existing
                      expected-partner crossing search; "collision" legalizes
                      crossings after A* collides with topology-allowed route
                      geometry; "lidar-pure" uses dynamic DRC-style crossing
                      permission against any committed route.
        min_straight_cells_per_crossing: Minimum straight access length on each
                      side of a crossing in grid cells.
        proactive_congestion_weight: Soft A* cost per blocked side-neighbor
                      cell beside straight moves.
        proactive_congestion_radius_cells: Sideways grid radius used for
                      proactive congestion counting.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        bend_radius_um: Minimum optical waveguide bend radius. Rounded up to
                      the active routing grid before primitive generation.
        use_indexed_heap: Benchmark-only indexed-heap experiment. Pass 8E
            measured it slower than duplicate-entry BinaryHeap queueing, so
            the default remains False.
        enable_simple_routes: If False, force optical nets through A* by
            disabling straight/L/Z simple-route candidates.
        primitive_ordering: Benchmark-only dense A* primitive ordering
            experiment. Pass 8F keeps "library" as the default.
        heuristic_mode: Dense A* heuristic experiment.
        heap_tie_breaker: Benchmark-only dense A* heap tie-breaker experiment.
            "smaller_g" preserves historical behavior; "larger_g" favors
            deeper states on equal f-score plateaus.
        max_iterations: Maximum A* state expansions per route attempt.
        routing_window_scale: Optional A* routing-window margin scale. If None,
                      the Rust AStarConfig default is used.
        include_heater_obstacles: If True, include configured heater/metal
                      layers as static optical-routing obstacles and enable
                      component-specific heater optical port openings.
        grid_size_um: Optical routing grid resolution in micrometers.
        waveguide_clearance_um: Static clearance in micrometers for existing
                      optical/waveguide obstacles.
        heater_clearance_um: Static clearance in micrometers for heater/metal
                      obstacles. Defaults to the waveguide clearance.
        obstacle_clearance_um: Deprecated alias for waveguide_clearance_um.
        chip_add_x_um: Extra horizontal chip margin added to both left and
                      right when the die bbox is computed automatically.
        chip_add_y_um: Extra vertical chip margin added to both bottom and top
                      when the die bbox is computed automatically.
        static_obstacle_config: Optional obstacle builder config. If omitted,
            strict bounding-box static obstacles are used.
        enable_electrical_routing: If True, run the electrical heater-metal
            routing stage after optical routing and return/write/show the
            electrically routed component.
        electrical_config: Optional electrical routing configuration. If
            omitted, `ElectricalRoutingConfig()` defaults are used.
        collect_route_stats: If True, collect route-search counters without
            printing debug timing. This is enabled automatically when stats is
            provided.
        collect_attempt_diagnostics: If True, collect extra per-attempt window,
            obstacle-density, and ripup diagnostics for slow/failed attempts.

    Returns:
        The routed layout component.
    """
    if show_routed is not None:
        show_klayout = bool(show_routed)
    if show_debug_svgs is not None:
        debug_svgs = show_debug_svgs
    if show_static_obstacles_svg is not None:
        debug_svgs = bool(show_static_obstacles_svg)
    if show_unrouted is not None:
        # Historical argument kept for compatibility.
        pass
    debug_svgs_enabled, debug_route_indices = _parse_debug_svg_selector(debug_svgs)
    total_steps = 4 if enable_electrical_routing else 3

    print(f"\n{'='*60}")
    print(f"Routing Flow: {benchmark_name}")
    print(f"{'='*60}")

    if stats is not None:
        stats.benchmark_name = benchmark_name

    if waveguide_clearance_um is None:
        waveguide_clearance_um = (
            float(obstacle_clearance_um)
            if obstacle_clearance_um is not None
            else SCRIPT_WAVEGUIDE_CLEARANCE_UM
        )
    if heater_clearance_um is None:
        heater_clearance_um = float(waveguide_clearance_um)

    route_static_obstacle_config = static_obstacle_config or StaticObstacleMapConfig(
        grid_size_um=float(grid_size_um),
        obstacle_mode="bounding_boxes",
        clearance_um=float(waveguide_clearance_um),
        heater_clearance_um=float(heater_clearance_um),
        chip_add_x_um=float(chip_add_x_um),
        chip_add_y_um=float(chip_add_y_um),
        clear_port_open_cells_from_static=False,
    )

    t_flow_start = time.perf_counter()

    if debug_svgs_enabled:
        prefix = benchmark_name.lower()
        for pattern in (
            f"build/static_obstacles/{prefix}_*.svg",
            f"build/routes/{prefix}_*.svg",
            f"build/routes/{prefix}_*_diagnostics.txt",
            f"build/routes/{prefix}_*_FAILED.txt",
            f"build/crossings/{prefix}_*.json",
            f"build/crossings/{prefix}_*.txt",
            f"build/electrical/{prefix}_*.svg",
        ):
            for path in Path(".").glob(pattern):
                try:
                    path.unlink()
                except OSError:
                    pass

    def _report_partial_debug_artifacts() -> None:
        if not debug_svgs_enabled:
            return
        prefix = benchmark_name.lower()
        build_dir = Path("build")
        obstacle_dir = build_dir / "static_obstacles"
        routes_dir = build_dir / "routes"
        electrical_dir = build_dir / "electrical"
        obstacle_svgs = sorted(obstacle_dir.glob(f"{prefix}_*.svg")) if obstacle_dir.exists() else []
        route_svgs = sorted(routes_dir.glob(f"{prefix}_*.svg")) if routes_dir.exists() else []
        electrical_svgs = (
            sorted(electrical_dir.glob(f"{prefix}_*.svg"))
            if electrical_dir.exists()
            else []
        )
        failed_logs = sorted(routes_dir.glob(f"{prefix}_*_FAILED.txt")) if routes_dir.exists() else []

        print("      - Partial debug artifacts:")
        print(f"        static obstacle SVGs: {len(obstacle_svgs)}")
        print(f"        route SVGs: {len(route_svgs)}")
        print(f"        electrical SVGs: {len(electrical_svgs)}")
        print(f"        failure logs: {len(failed_logs)}")
        for failed_log in failed_logs:
            print(f"        failure log: {failed_log}")

        try:
            for svg_path in obstacle_svgs:
                webbrowser.open_new_tab(svg_path.resolve().as_uri())
            for svg_path in route_svgs:
                webbrowser.open_new_tab(svg_path.resolve().as_uri())
            for svg_path in electrical_svgs:
                webbrowser.open_new_tab(svg_path.resolve().as_uri())
        except Exception as e:
            print(f"      - Warning: failed to open partial SVGs automatically: {e}")

    # Step 1: Load benchmark
    step_load_start = time.perf_counter()
    print(f"\n[1/{total_steps}] Loading benchmark: {benchmark_name}...")
    schematic = load_benchmark(benchmark_name)
    step_load_end = time.perf_counter()
    if stats is not None:
        stats.instance_count = len(schematic.netlist.instances)
        stats.net_count = len(schematic.netlist.routes)
        stats.step_times_s["load_benchmark"] = step_load_end - step_load_start
    print("      ✓ Schematic loaded")
    if debug_meanders:
        print(f"      - Instances: {list(schematic.netlist.instances.keys())}")
        print(f"      - Placements: {list(schematic.placements.keys())}")
    else:
        print(f"      - Instances: {len(schematic.netlist.instances)}")
        print(f"      - Placements: {len(schematic.placements)}")

    # Step 2: Translate schematic to layout
    step_layout_start = time.perf_counter()
    print(f"\n[2/{total_steps}] Translating schematic to layout...")
    unrouted_layout = layout_from_schematic(schematic)
    step_layout_end = time.perf_counter()
    if stats is not None:
        stats.step_times_s["layout_from_schematic"] = step_layout_end - step_layout_start
    print(f"      ✓ Layout generated: {unrouted_layout.name}")
    bbox = unrouted_layout.bbox
    if callable(bbox):
        bbox = bbox()
    print(f"      - Bounding box: {bbox}")
    if debug_timing:
        print(f"      - Translation time: {step_layout_end - step_layout_start:.4f} s")

    # Step 3: Route nets with Rust backend
    print(f"\n[3/{total_steps}] Routing nets with Rust backend...")
    if stats is not None:
        stats.step_times_s["build_static_obstacle_map"] = 0.0
        stats.step_times_s["baseline_gdsfactory_routing"] = 0.0
    debug_dir = Path("build") if debug_svgs_enabled else None
    metadata = load_benchmark_metadata(benchmark_name, schematic=schematic)
    t_route_start = time.perf_counter()
    try:
        route_result = route_match_and_realize(
            unrouted_layout,
            schematic,
            enable_path_length_matching=enable_path_length_matching,
            path_length_match_outputs=path_length_match_outputs,
            node_types=metadata.get("node_types"),
            internal_delays_um=metadata.get("internal_delays_um"),
            enable_crossings=enable_crossings,
            crossing_mode=crossing_mode,
            crossing_half_size_cells=int(crossing_half_size_cells),
            min_straight_cells_per_crossing=int(min_straight_cells_per_crossing),
            foreign_port_keepout_cells=int(foreign_port_keepout_cells),
            node_depths=metadata.get("node_depths"),
            node_ranks=metadata.get("node_ranks"),
            edge_ranks=metadata.get("edge_ranks"),
            debug_dir=debug_dir,
            debug_prefix=benchmark_name.lower(),
            debug_route_indices=debug_route_indices,
            debug_stop_after_route_index=debug_stop_after_route_index,
            debug_timing=debug_timing,
            verbose_route_diagnostics=verbose_routes or debug_meanders,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_um=bend_radius_um,
            enable_jps4=enable_jps4,
            use_indexed_heap=use_indexed_heap,
            enable_simple_routes=enable_simple_routes,
            primitive_ordering=primitive_ordering,
            heuristic_mode=heuristic_mode,
            heap_tie_breaker=heap_tie_breaker,
            proactive_congestion_weight=float(proactive_congestion_weight),
            proactive_congestion_radius_cells=int(proactive_congestion_radius_cells),
            max_iterations=max_iterations,
            routing_window_scale=routing_window_scale,
            collect_route_stats=collect_route_stats or stats is not None,
            collect_attempt_diagnostics=collect_attempt_diagnostics,
            include_heater_obstacles=include_heater_obstacles,
            ripup_reroute_config=ripup_reroute_config,
            path_length_meander_height_um=path_length_meander_height_um,
            enable_grid_endpoint_correction=True,
            obstacle_config=route_static_obstacle_config,
        )
    except Exception:
        print("      ✗ Routing failed.")
        _report_partial_debug_artifacts()
        raise
    routed_layout = route_result.routed_layout
    debug_artifacts = route_result.debug_artifacts
    t_route_end = time.perf_counter()
    route_summary = debug_artifacts.route_search_summary
    route_attempt_records = [
        record_dict
        for record in getattr(debug_artifacts, "route_attempt_records", ())
        if (record_dict := _route_attempt_as_dict(record))
    ]
    if stats is not None:
        route_time = t_route_end - t_route_start
        stats.step_times_s["baseline_gdsfactory_routing"] = route_time
        if "build_static_obstacle_map" not in stats.step_times_s:
            stats.step_times_s["build_static_obstacle_map"] = 0.0
        for name, elapsed_s in getattr(route_result, "pipeline_timings_s", {}).items():
            stats.step_times_s[str(name)] = float(elapsed_s)
        if debug_artifacts.realization_grid_spec is not None:
            width, height, *_ = debug_artifacts.realization_grid_spec
            stats.static_grid_width = int(width)
            stats.static_grid_height = int(height)
        blocked_count = len(debug_artifacts.static_blocked_cells)
        if blocked_count == 0:
            blocked_count = int(getattr(debug_artifacts, "static_obstacle_count", 0) or 0)
        if blocked_count > 0:
            stats.blocked_cells = blocked_count
            stats.raw_blocked_cells = blocked_count
            port_open_count = int(
                getattr(debug_artifacts, "static_port_open_count", 0) or 0
            )
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
        stats.skipped_duplicate_heap_entries = int(
            route_summary.skipped_duplicate_heap_entries
        )
        stats.stale_generation_heap_entries = int(
            route_summary.stale_generation_heap_entries
        )
        stats.closed_heap_entries = int(route_summary.closed_heap_entries)
        stats.max_heap_size = int(route_summary.max_heap_size)
        stats.dense_search_states = int(route_summary.dense_search_states)
        stats.dense_search_storage_bytes = int(
            route_summary.dense_search_storage_bytes
        )
        stats.best_cost_updates = int(route_summary.best_cost_updates)
        stats.parent_updates = int(route_summary.parent_updates)
        stats.obstacle_clearance_checks = int(route_summary.obstacle_clearance_checks)
        stats.footprint_checks = int(route_summary.footprint_checks)
        stats.footprint_rect_checks = int(route_summary.footprint_rect_checks)
        stats.crossing_candidate_checks = int(route_summary.crossing_candidate_checks)
        stats.crossing_accepted = int(route_summary.crossing_accepted)
        stats.crossing_reject_non_straight = int(
            route_summary.crossing_reject_non_straight
        )
        stats.crossing_reject_not_perpendicular = int(
            route_summary.crossing_reject_not_perpendicular
        )
        stats.crossing_reject_margin = int(route_summary.crossing_reject_margin)
        stats.crossing_reject_wrong_order = int(route_summary.crossing_reject_wrong_order)
        stats.crossing_reject_unexpected_owner = int(
            route_summary.crossing_reject_unexpected_owner
        )
        stats.crossing_reject_unmatched_owner = int(
            route_summary.crossing_reject_unmatched_owner
        )
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
        stats.crossing_reject_pending_straight = int(
            route_summary.crossing_reject_pending_straight
        )
        stats.full_grid_fallbacks = int(route_summary.full_grid_fallbacks)
        stats.neighbor_generation_time_s = (
            float(route_summary.neighbor_generation_time_us) / 1_000_000.0
        )
        stats.heap_operation_time_s = (
            float(route_summary.heap_operation_time_us) / 1_000_000.0
        )
        stats.legality_check_time_s = (
            float(route_summary.legality_check_time_us) / 1_000_000.0
        )
        stats.reconstruction_time_s = (
            float(route_summary.reconstruction_time_us) / 1_000_000.0
        )
        stats.search_loop_time_s = (
            float(route_summary.search_loop_time_us) / 1_000_000.0
        )
        stats.obstacle_map_prepare_time_s = (
            float(route_summary.obstacle_map_prepare_time_us) / 1_000_000.0
        )
        stats.simple_route_time_s = (
            float(route_summary.simple_route_time_us) / 1_000_000.0
        )
        stats.commit_prepare_time_s = (
            float(route_summary.commit_prepare_time_us) / 1_000_000.0
        )
        stats.commit_time_s = float(route_summary.commit_time_us) / 1_000_000.0
        stats.route_attempt_records = route_attempt_records
    if debug_timing:
        route_time = t_route_end - t_route_start
        timings = getattr(route_result, "pipeline_timings_s", {})
        route_nets_time = float(timings.get("route_nets", 0.0))
        plm_analysis_time = float(timings.get("path_length_analysis", 0.0))
        plm_obstacle_time = float(timings.get("meander_obstacle_map", 0.0))
        plm_planning_time = float(timings.get("meander_planning", 0.0))
        route_endpoint_correction_time = float(
            timings.get("route_endpoint_correction", 0.0)
        )
        realization_time = float(timings.get("route_realization", 0.0))
        plm_total = plm_analysis_time + plm_obstacle_time + plm_planning_time
        known_substage_time = (
            route_nets_time
            + route_endpoint_correction_time
            + plm_total
            + realization_time
        )
        overhead_time = max(0.0, route_time - known_substage_time)
        print(
            "      - Optical routing stage time "
            f"(net routing + PLM + realization): {route_time:.4f} s"
        )
        print(
            "        - net routing phase "
            f"(obstacles + A* + repairs): {route_nets_time:.4f} s"
        )
        route_nets_subtimings = {
            str(name).removeprefix("route_nets."): float(elapsed_s)
            for name, elapsed_s in timings.items()
            if str(name).startswith("route_nets.")
        }
        if route_nets_subtimings:
            ordered_subtiming_names = (
                "obstacle_map",
                "router_setup",
                "route_job_build",
                "port_opening_prep",
                "port_opening_batch",
                "static_map_handoff",
                "state_opening_precompute",
                "clearance_exempt_batch",
                "batch_job_pack",
                "native_route_batch",
                "batch_result_processing",
                "endpoint_correction_pack",
                "endpoint_correction_native",
                "endpoint_correction_processing",
                "record_assembly",
                "direct_realization",
                "debug_artifact_assembly",
            )
            known_route_nets_s = sum(route_nets_subtimings.values())
            parts = [
                f"{name}={route_nets_subtimings[name]:.4f}s"
                for name in ordered_subtiming_names
                if route_nets_subtimings.get(name, 0.0) > 0.0
            ]
            route_nets_other_s = max(0.0, route_nets_time - known_route_nets_s)
            if route_nets_other_s > 1.0e-4:
                parts.append(f"other={route_nets_other_s:.4f}s")
            print("          route_nets split: " + ", ".join(parts))
        print(
            "          route search: "
            f"astar_loop={float(route_summary.astar_elapsed_s):.4f}s, "
            f"attempts={int(route_summary.route_attempts)}, "
            f"failures={int(route_summary.route_failures)}, "
            f"simple={int(route_summary.simple_route_count)}/"
            f"{int(route_summary.route_count)}, "
            f"repairs={int(route_summary.repair_count)}"
        )
        endpoint_correction_time_s = float(
            getattr(route_summary, "endpoint_correction_time_s", 0.0)
        )
        if endpoint_correction_time_s > 0.0:
            print(
                "          endpoint correction: "
                f"time={endpoint_correction_time_s:.4f}s, "
                f"calls={int(getattr(route_summary, 'endpoint_correction_calls', 0))}, "
                f"failures={int(getattr(route_summary, 'endpoint_correction_failures', 0))}"
            )
        print(
            "          A* counters: "
            f"expanded={int(route_summary.expanded_states)}, "
            f"generated={int(route_summary.generated_neighbors)}, "
            f"heap_pushes={int(route_summary.heap_pushes)}, "
            f"heap_pops={int(route_summary.heap_pops)}, "
            f"footprint_checks={int(route_summary.footprint_checks)}, "
            f"rect_checks={int(route_summary.footprint_rect_checks)}, "
            f"full_grid_fallbacks={int(route_summary.full_grid_fallbacks)}"
        )
        print(
            "          A* timed ops: "
            f"dense_build={float(route_summary.dense_grid_build_time_us) / 1_000_000.0:.4f}s, "
            f"search_loop={float(route_summary.search_loop_time_us) / 1_000_000.0:.4f}s, "
            f"obstacle_prepare={float(route_summary.obstacle_map_prepare_time_us) / 1_000_000.0:.4f}s, "
            f"simple_probe={float(route_summary.simple_route_time_us) / 1_000_000.0:.4f}s, "
            f"commit_prepare={float(route_summary.commit_prepare_time_us) / 1_000_000.0:.4f}s, "
            f"commit={float(route_summary.commit_time_us) / 1_000_000.0:.4f}s, "
            f"neighbor={float(route_summary.neighbor_generation_time_us) / 1_000_000.0:.4f}s, "
            f"heap={float(route_summary.heap_operation_time_us) / 1_000_000.0:.4f}s, "
            f"legality={float(route_summary.legality_check_time_us) / 1_000_000.0:.4f}s, "
            f"reconstruction={float(route_summary.reconstruction_time_us) / 1_000_000.0:.4f}s"
        )
        timed_search_s = float(route_summary.search_loop_time_us) / 1_000_000.0
        measured_inner_s = (
            float(route_summary.neighbor_generation_time_us)
            + float(route_summary.heap_operation_time_us)
            + float(route_summary.legality_check_time_us)
        ) / 1_000_000.0
        route_overhead_s = (
            float(route_summary.obstacle_map_prepare_time_us)
            + float(route_summary.simple_route_time_us)
            + float(route_summary.commit_prepare_time_us)
            + float(route_summary.commit_time_us)
        ) / 1_000_000.0
        if timed_search_s > 0.0:
            print(
                "          A* loop attribution: "
                f"measured_inner={measured_inner_s:.4f}s, "
                f"other={max(0.0, timed_search_s - measured_inner_s):.4f}s, "
                f"route_overhead={route_overhead_s:.4f}s"
            )
        slowest_net_lines = _format_slowest_route_net_lines(
            route_attempt_records,
            limit=8,
        )
        if slowest_net_lines:
            print("          slowest route nets:")
            for line in slowest_net_lines:
                print(line)
        slowest_attempt_lines = _format_slowest_route_attempt_lines(
            route_attempt_records,
            limit=8,
        )
        if slowest_attempt_lines:
            print("          slowest route attempts:")
            for line in slowest_attempt_lines:
                print(line)
        if plm_total > 0.0:
            print(
                "        - path-length matching phase: "
                f"{plm_total:.4f} s "
                f"(analysis={plm_analysis_time:.4f}s, "
                f"meander_obstacles={plm_obstacle_time:.4f}s, "
                f"meander_planning={plm_planning_time:.4f}s)"
            )
        if route_endpoint_correction_time > 0.0:
            print(
                "        - route endpoint correction phase: "
                f"{route_endpoint_correction_time:.4f} s"
            )
        print(f"        - route realization phase: {realization_time:.4f} s")
        if overhead_time > 1.0e-3:
            print(f"        - stage overhead/reporting: {overhead_time:.4f} s")
    print(f"      ✓ Routed layout generated: {routed_layout.name}")
    electrical_result: ElectricalRoutingResult | None = None

    crossing_plan_info = getattr(debug_artifacts, "crossing_plan_info", None)
    if crossing_plan_info is not None:
        routed_layout.info["crossing_plan"] = crossing_plan_info

    if route_result.path_length_analysis_info is not None:
        meander_report_info = getattr(route_result, "meander_insertion_report_info", None)
        routed_layout.info["path_length_analysis"] = route_result.path_length_analysis_info
        routed_layout.info["meander_requirements"] = (
            route_result.meander_requirements_info or []
        )
        if meander_report_info is not None:
            routed_layout.info["meander_insertion_report"] = (
                meander_report_info
            )
        print(
            "      - Path-length matching: "
            f"{len(routed_layout.info['meander_requirements'])} edge(s) require extra length"
        )
        group_diagnostics = route_result.path_length_analysis_info.get(
            "matching_group_diagnostics",
            route_result.path_length_analysis_info.get("matching_groups", []),
        )
        if isinstance(group_diagnostics, list):
            groups_over_tolerance = sum(
                1
                for group in group_diagnostics
                if isinstance(group, dict) and group.get("within_tolerance") is False
            )
            max_residual = max(
                (
                    float(group.get("max_accepted_unmatched_um", 0.0))
                    for group in group_diagnostics
                    if isinstance(group, dict)
                ),
                default=0.0,
            )
            print(
                "      - Path-length groups: "
                f"{len(group_diagnostics)} group(s), "
                f"over_tolerance={groups_over_tolerance}, "
                f"max_residual={max_residual:.6f}um"
            )
        if debug_meanders and route_result.path_length_analysis_info is not None:
            node_timings = route_result.path_length_analysis_info.get("node_timings_um", {})
            if isinstance(node_timings, dict):
                for node_name, node_info in node_timings.items():
                    if not isinstance(node_info, dict):
                        continue
                    incoming = node_info.get("incoming_edges")
                    if incoming is None:
                        incoming = []
                    print(
                        f"        • node={node_name}, "
                        f"type={node_info.get('node_type')}, "
                        f"internal={float(node_info.get('internal_delay_um', 0.0)):.3f}um, "
                        f"input={float(node_info.get('input_arrival_um', 0.0)):.3f}um, "
                        f"output={float(node_info.get('output_arrival_um', 0.0)):.3f}um"
                    )
                    for incoming_entry in incoming:
                        if not isinstance(incoming_entry, dict):
                            continue
                        edge = incoming_entry.get("edge", {})
                        edge_name = (
                            f"{edge.get('source', {}).get('instance', '?')}->"
                            f"{edge.get('target', {}).get('instance', '?')} "
                            f"({edge.get('net_name', '?')})"
                        )
                        print(
                            "          - "
                            f"{edge_name}: edge_len={float(incoming_entry.get('routed_length_um', 0.0)):.3f}um, "
                            f"edge_arrival={float(incoming_entry.get('edge_arrival_um', 0.0)):.3f}um, "
                            f"missing={float(incoming_entry.get('missing_length_um', 0.0)):.3f}um"
                        )
        if meander_report_info is not None:
            report = meander_report_info
            total_requested = float(report.get("total_requested_extra_length_um", 0.0))
            total_inserted = float(report.get("total_inserted_extra_length_um", 0.0))
            unmatched = float(report.get("unmatched_length_um", 0.0))
            print(
                "      - Meander insertion: "
                f"requested={total_requested:.3f}um, "
                f"inserted={total_inserted:.3f}um, "
                f"unmatched={unmatched:.3f}um"
            )
            if debug_meanders:
                setup_profile = report.get("setup_profile", {})
                if isinstance(setup_profile, dict) and setup_profile:
                    print(
                        "        Meander setup profile: "
                        f"total={float(setup_profile.get('total_s', 0.0)):.4f}s, "
                        f"router_init={float(setup_profile.get('router_init_s', 0.0)):.4f}s, "
                        f"by_edge={float(setup_profile.get('by_edge_s', 0.0)):.4f}s, "
                        f"base_static_collect={float(setup_profile.get('base_static_collect_s', 0.0)):.4f}s, "
                        f"base_static_reused={int(float(setup_profile.get('base_static_reused', 0.0)))}, "
                        f"static_handle={int(float(setup_profile.get('combined_static_route_registration_handle', 0.0)))}, "
                        f"set_static={float(setup_profile.get('set_static_cells_s', 0.0)):.4f}s, "
                        f"register_routes={float(setup_profile.get('register_route_cells_s', 0.0)):.4f}s, "
                        f"register_geometry={float(setup_profile.get('register_route_geometry_s', 0.0)):.4f}s, "
                        f"registered_records={int(float(setup_profile.get('registered_record_count', 0.0)))}, "
                        f"unregistered_records={int(float(setup_profile.get('unregistered_record_count', 0.0)))}, "
                        f"unregistered_route_static={int(float(setup_profile.get('unregistered_route_static_cell_count', 0.0)))}, "
                        f"route_occupancy_radius={int(float(setup_profile.get('route_occupancy_radius_cells', 0.0)))}, "
                        f"box_clearance_radius={int(float(setup_profile.get('meander_box_clearance_radius_cells', 0.0)))}, "
                        f"unique_route_cells={int(float(setup_profile.get('unique_route_cell_count', 0.0)))}"
                    )
                    print(
                        "        Meander route-registration setup split: "
                        f"edge_order={float(setup_profile.get('edge_order_s', 0.0)):.4f}s, "
                        f"route_objects={float(setup_profile.get('route_object_list_s', 0.0)):.4f}s, "
                        f"base_static_list={float(setup_profile.get('base_static_registration_list_s', 0.0)):.4f}s, "
                        f"rust_call={float(setup_profile.get('register_route_cells_call_s', 0.0)):.4f}s, "
                        f"result_map={float(setup_profile.get('registration_result_map_s', 0.0)):.4f}s"
                    )
                    print(
                        "        Meander geometry-registration setup split: "
                        f"prepare={float(setup_profile.get('geometry_prepare_s', 0.0)):.4f}s, "
                        f"centerline_copy={float(setup_profile.get('geometry_centerline_copy_s', 0.0)):.4f}s, "
                        f"max_bumps={float(setup_profile.get('geometry_max_bumps_s', 0.0)):.4f}s, "
                        f"rust_call={float(setup_profile.get('geometry_call_s', 0.0)):.4f}s, "
                        f"result_map={float(setup_profile.get('geometry_result_map_s', 0.0)):.4f}s"
                    )
                    if any(
                        key.startswith("rust_registration_")
                        for key in setup_profile
                    ):
                        print(
                            "        Rust route-registration split: "
                            f"total={float(setup_profile.get('rust_registration_total_s', 0.0)):.4f}s, "
                            f"reset={float(setup_profile.get('rust_registration_reset_s', 0.0)):.4f}s, "
                            f"base_pack={float(setup_profile.get('rust_registration_base_static_pack_s', 0.0)):.4f}s, "
                            f"base_obstacles={float(setup_profile.get('rust_registration_base_static_obstacle_add_s', 0.0)):.4f}s, "
                            f"base_prefix={float(setup_profile.get('rust_registration_base_prefix_build_s', 0.0)):.4f}s, "
                            f"route_extract={float(setup_profile.get('rust_registration_route_extract_s', 0.0)):.4f}s, "
                            f"route_cells={float(setup_profile.get('rust_registration_route_cell_collect_s', 0.0)):.4f}s, "
                            f"open_sets={float(setup_profile.get('rust_registration_open_set_build_s', 0.0)):.4f}s, "
                            f"route_list={float(setup_profile.get('rust_registration_route_cell_list_s', 0.0)):.4f}s, "
                            f"route_static={float(setup_profile.get('rust_registration_route_static_add_s', 0.0)):.4f}s, "
                            f"store={float(setup_profile.get('rust_registration_registered_store_s', 0.0)):.4f}s, "
                            f"routes={int(float(setup_profile.get('rust_registration_route_count', 0.0)))}, "
                            f"base_static={int(float(setup_profile.get('rust_registration_base_static_cell_count', 0.0)))}, "
                            f"unique_route={int(float(setup_profile.get('rust_registration_unique_route_cell_count', 0.0)))}, "
                            f"open_cells={int(float(setup_profile.get('rust_registration_registered_open_cell_count', 0.0)))}"
                        )
                print(
                    "        Meander overhead profile: "
                    f"planner={float(report.get('planner_elapsed_s', 0.0)):.4f}s, "
                    f"candidate_setup={float(report.get('candidate_overhead_s', 0.0)):.4f}s, "
                    f"commit={float(report.get('commit_elapsed_s', 0.0)):.4f}s"
                )
                rust_planner_profile = report.get("rust_planner_profile", {})
                if isinstance(rust_planner_profile, dict) and rust_planner_profile:
                    print(
                        "        Rust meander planner split: "
                        f"total={float(rust_planner_profile.get('total_s', 0.0)):.4f}s, "
                        f"free_interval={float(rust_planner_profile.get('free_interval_s', 0.0)):.4f}s, "
                        f"box_check={float(rust_planner_profile.get('box_check_s', 0.0)):.4f}s, "
                        f"analytic_plan={float(rust_planner_profile.get('analytic_plan_s', 0.0)):.4f}s, "
                        f"replacement_check={float(rust_planner_profile.get('replacement_check_s', 0.0)):.4f}s, "
                        f"footprint={float(rust_planner_profile.get('footprint_s', 0.0)):.4f}s, "
                        f"run_extraction={float(rust_planner_profile.get('run_extraction_s', 0.0)):.4f}s, "
                        f"plan_calls={int(float(rust_planner_profile.get('plan_calls', 0.0)))}, "
                        f"depths={int(float(rust_planner_profile.get('depth_count', 0.0)))}, "
                        f"run_side_checks={int(float(rust_planner_profile.get('run_side_checks', 0.0)))}, "
                        f"box_checks={int(float(rust_planner_profile.get('box_checks', 0.0)))}, "
                        f"analytic_calls={int(float(rust_planner_profile.get('analytic_plan_calls', 0.0)))}"
                    )
                rust_wrapper_profile = report.get("rust_wrapper_profile", {})
                if isinstance(rust_wrapper_profile, dict) and rust_wrapper_profile:
                    print(
                        "        Rust meander wrapper split: "
                        f"planner_call={float(rust_wrapper_profile.get('planner_call_s', 0.0)):.4f}s, "
                        f"reserved_snapshot={float(rust_wrapper_profile.get('reserved_snapshot_s', 0.0)):.4f}s, "
                        f"rect_cells={float(rust_wrapper_profile.get('selected_rect_cells_s', 0.0)):.4f}s, "
                        f"reserved_update={float(rust_wrapper_profile.get('candidate_reserved_update_s', 0.0)):.4f}s, "
                        f"py_plan={float(rust_wrapper_profile.get('py_plan_conversion_s', 0.0)):.4f}s, "
                        f"py_candidate_result={float(rust_wrapper_profile.get('py_candidate_result_build_s', 0.0)):.4f}s, "
                        f"py_result={float(rust_wrapper_profile.get('py_result_build_s', 0.0)):.4f}s, "
                        f"prepare_calls={int(float(rust_wrapper_profile.get('extra_blocked_prepare_calls', 0.0)))}, "
                        f"rect_cells_count={int(float(rust_wrapper_profile.get('selected_rect_cell_count', 0.0)))}, "
                        f"py_plans={int(float(rust_wrapper_profile.get('py_plan_count', 0.0)))}, "
                        f"candidate_results={int(float(rust_wrapper_profile.get('candidate_result_count', 0.0)))}"
                    )
                commit_profile = report.get("commit_profile", {})
                if isinstance(commit_profile, dict) and commit_profile:
                    sorted_commit = sorted(
                        commit_profile.items(),
                        key=lambda item: -float(item[1]),
                    )
                    commit_parts = [
                        f"{key[:-2] if key.endswith('_s') else key}={float(value):.4f}s"
                        for key, value in sorted_commit
                    ]
                    print(
                        "        Meander commit split: "
                        + ", ".join(commit_parts)
                    )
                print(
                    "        Meander candidate execution: "
                    f"requirement_batches={int(report.get('requirement_batch_calls', 0))}, "
                    f"requirement_batch_candidates={int(report.get('requirement_batch_candidate_calls', 0))}, "
                    f"requirement_batch_edge_calls={int(report.get('requirement_batch_edge_calls', 0))}, "
                    f"bundle_candidates={int(report.get('bundle_candidate_calls', 0))}, "
                    f"bundle_edge_calls={int(report.get('bundle_edge_calls', 0))}, "
                    f"bundle_planned={int(report.get('bundle_planned', 0))}, "
                    f"bundle_no_candidate={int(report.get('bundle_no_candidate', 0))}"
                )
                candidate_engine_counts = report.get("candidate_engine_counts", {})
                if isinstance(candidate_engine_counts, dict) and candidate_engine_counts:
                    formatted_engine_counts = ", ".join(
                        f"{key}={int(value)}"
                        for key, value in sorted(candidate_engine_counts.items())
                        if isinstance(key, str) and isinstance(value, (int, float))
                    )
                    if formatted_engine_counts:
                        print(
                            "        Meander candidate engines: "
                            f"{formatted_engine_counts}"
                        )
                candidate_setup_profile = report.get("candidate_setup_profile", {})
                if isinstance(candidate_setup_profile, dict) and candidate_setup_profile:
                    sorted_setup = sorted(
                        candidate_setup_profile.items(),
                        key=lambda item: -float(item[1]),
                    )
                    setup_parts = [
                        f"{key[:-2] if key.endswith('_s') else key}={float(value):.4f}s"
                        for key, value in sorted_setup
                    ]
                    print(
                        "        Candidate setup split: "
                        + ", ".join(setup_parts)
                    )
                candidate_profile = report.get("candidate_profile", {})
                if isinstance(candidate_profile, dict) and candidate_profile:
                    print("        Candidate planner profile:")
                    sorted_profile = sorted(
                        candidate_profile.items(),
                        key=lambda item: (
                            -float(item[1].get("elapsed_s", 0.0))
                            if isinstance(item[1], dict)
                            else 0.0
                        ),
                    )
                    for reason, raw_profile in sorted_profile:
                        if not isinstance(raw_profile, dict):
                            continue
                        print(
                            "          - "
                            f"{reason}: candidates={int(raw_profile.get('candidate_attempts', 0))}, "
                            f"edge_calls={int(raw_profile.get('edge_calls', 0))}, "
                            f"planned={int(raw_profile.get('planned', 0))}, "
                            f"no_candidate={int(raw_profile.get('no_candidate', 0))}, "
                            f"elapsed={float(raw_profile.get('elapsed_s', 0.0)):.4f}s"
                        )
                for entry in report.get("results", []):
                    edge = entry.get("edge", {})
                    net_name = edge.get("net_name", "<unknown>")
                    status = entry.get("status", "<unknown>")
                    reason = entry.get("reason", "")
                    req = float(entry.get("requested_extra_length_um", 0.0))
                    ins = float(entry.get("inserted_extra_length_um", 0.0))
                    unmatched = float(entry.get("unmatched_length_um", max(0.0, req - ins)))
                    planning_mode = entry.get("planning_mode", None)
                    effective_radius = entry.get("effective_bend_radius_um", None)
                    primitive_radius = entry.get("primitive_bend_radius_um", None)
                    selected_box = entry.get("selected_box", None)
                    selected_grid_rect = entry.get("selected_grid_rect", None)
                    bumps = entry.get("bumps", None)
                    visual_bumps = entry.get("visual_bumps", None)
                    u_turns = entry.get("u_turns", None)
                    quarter_turns = entry.get("quarter_turns", None)
                    side = entry.get("side", None)
                    reserved_cells_count = entry.get("reserved_cells_count", None)
                    print(
                        f"        • {net_name}: status={status}, requested={req:.3f}um, "
                        f"inserted={ins:.3f}um, unmatched={unmatched:.3f}um, "
                        f"planning_mode={planning_mode}, side={side}, bumps={bumps}, "
                        f"visual_bumps={visual_bumps}, u_turns={u_turns}, "
                        f"quarter_turns={quarter_turns}, "
                        f"effective_bend_radius_um={effective_radius}, "
                        f"primitive_bend_radius_um={primitive_radius}, "
                        f"selected_box={selected_box}, selected_grid_rect={selected_grid_rect}, "
                        f"reserved_cells_count={reserved_cells_count}, reason={reason}"
                    )

    if enable_electrical_routing:
        print(f"\n[4/{total_steps}] Routing heater electrical metal...")
        t_electrical_start = time.perf_counter()
        current_electrical_result = route_electrical_heaters(
            routed_layout,
            schematic,
            electrical_config,
            debug_dir=debug_dir,
            debug_prefix=benchmark_name.lower(),
        )
        electrical_result = current_electrical_result
        t_electrical_end = time.perf_counter()
        if stats is not None:
            stats.step_times_s["electrical_routing"] = (
                t_electrical_end - t_electrical_start
            )
        if current_electrical_result.routed_component is None:
            raise RuntimeError(_electrical_failure_summary(current_electrical_result))
        electrical_summary = _electrical_summary(
            current_electrical_result,
            electrical_config,
        )
        if stats is not None:
            stats.electrical_terminal_groups = int(
                electrical_summary["terminal_group_count"]
            )
            stats.electrical_pad_assignments = int(
                electrical_summary["pad_assignment_count"]
            )
            stats.electrical_detailed_routes = int(
                electrical_summary["detailed_route_count"]
            )
            stats.electrical_failed_detailed_routes = int(
                electrical_summary["failed_detailed_route_count"]
            )
        optical_routed_layout = routed_layout
        routed_layout = current_electrical_result.routed_component
        _copy_component_info(optical_routed_layout, routed_layout)
        _component_info(routed_layout)["electrical_routing"] = electrical_summary
        electrical_pad_count = (
            len(current_electrical_result.pad_plan.assignments)
            if current_electrical_result.pad_plan
            else 0
        )
        if current_electrical_result.terminal_groups:
            print(f"      ✓ Electrical layout generated: {routed_layout.name}")
        else:
            print("      ✓ No heater electrical terminals found; electrical routing skipped")
        print(
            "      - Electrical routes: "
            f"heaters={len(current_electrical_result.terminal_groups)}, "
            f"pads={electrical_pad_count}"
        )
        if debug_timing:
            print(
                "      - Electrical routing time: "
                f"{t_electrical_end - t_electrical_start:.4f} s"
            )
        if debug_svgs_enabled:
            for name, path in current_electrical_result.debug_artifacts.items():
                print(f"      - Electrical {name}: {path}")

    if debug_svgs_enabled:
        if debug_artifacts.obstacle_svg is not None:
            print(f"      - Obstacle SVG: {debug_artifacts.obstacle_svg}")
        if debug_route_indices is None:
            if debug_artifacts.route_svgs:
                print(f"      - Route SVGs: {len(debug_artifacts.route_svgs)} files")
        else:
            selected = _format_debug_route_indices(debug_route_indices)
            print(
                f"      - Route SVGs: {len(debug_artifacts.route_svgs)} "
                f"selected file(s), route indices: {selected}"
            )

        # Open generated SVGs in the default browser/viewer so the user can inspect them.
        try:
            if debug_artifacts.obstacle_svg is not None:
                obs_path = Path(debug_artifacts.obstacle_svg)
                if obs_path.exists():
                    webbrowser.open_new_tab(obs_path.resolve().as_uri())
            for svg in debug_artifacts.route_svgs or []:
                svg_path = Path(svg)
                if svg_path.exists():
                    webbrowser.open_new_tab(svg_path.resolve().as_uri())
            if electrical_result is not None:
                for svg in electrical_result.debug_artifacts.values():
                    svg_path = Path(svg)
                    if svg_path.exists():
                        webbrowser.open_new_tab(svg_path.resolve().as_uri())
        except Exception as e:
            print(f"      - Warning: failed to open SVGs automatically: {e}")

    # Optionally show the final routed layout in KLayout
    if show_klayout:
        try:
            print("      - Opening routed layout in KLayout...")
            routed_layout.show()
        except Exception as e:
            print(f"      - Warning: failed to open layout in KLayout: {e}")
    else:
        print("      - Write GDS...")
        routed_layout.write_gds(f"build/routed_{benchmark_name}.gds")

    if debug_timing:
        t_end = time.perf_counter()
        total = t_end - t_flow_start
        print(f"\nTiming summary for {benchmark_name}:\n  total: {total:.4f} s")
    if stats is not None:
        total = time.perf_counter() - t_flow_start
        stats.total_time_s = float(total)

    print(f"\n{'='*60}\n")

    return routed_layout


if __name__ == "__main__":
    main()
