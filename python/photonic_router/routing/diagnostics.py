"""Phase 6 diagnostics: the route, attempt and failure write-ups behind the debug path.

Nothing in here takes a routing decision. `dispatch.py` calls these writers
when `state.obstacles.debug_path` or the attempt-diagnostics flag is set; the two small
state readers `state_openings_for_job` and `route_cells_from_router` live
here because both the dispatch and the writers need them.
"""

from __future__ import annotations

import contextlib
import math
from collections import deque
from collections.abc import Mapping
from typing import Any
from xml.sax.saxutils import escape as _xml_escape

from photonic_router.gds.route_rust_debug_artifacts import (
    cells_bbox,
    ensure_dir,
    rect_cell_count,
    rect_overlap_cell_count,
    route_cells_bbox,
)
from photonic_router.records.route_rust_geometry import (
    format_illegal_crossing_root_causes_line,
    format_native_repair_trace_lines,
)
from photonic_router.records.route_rust_types import (
    RouteJob,
    as_float,
)
from photonic_router.routing.handoff import (
    states_and_openings,
)
from photonic_router.routing.products import StateOpenings
from photonic_router.routing.route_jobs import angle_to_step
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.state import SessionState


def _direction_reaches_target_ray(
    *,
    source: tuple[int, int, int],
    target_cell: tuple[int, int],
    tolerance: int,
) -> bool:
    """Whether the heading of ``source`` (x, y, angle) points at ``target_cell``."""
    source_x, source_y, source_angle = source
    target_x, target_y = target_cell
    dx = target_x - source_x
    dy = target_y - source_y
    if abs(dx) <= tolerance and abs(dy) <= tolerance:
        return True
    dir_x, dir_y = angle_to_step(source_angle)
    if dir_x == 0 and dir_y == 0:
        return False
    if dir_x == 0:
        return abs(dx) <= tolerance and (dy > 0) == (dir_y > 0)
    if dir_y == 0:
        return abs(dy) <= tolerance and (dx > 0) == (dir_x > 0)
    return (
        (dx > 0) == (dir_x > 0) and (dy > 0) == (dir_y > 0) and abs(abs(dx) - abs(dy)) <= tolerance
    )


def _source_lower_bounds(
    settings: SessionSettings,
    state: SessionState,
    *,
    source: tuple[int, int, int],
    target: tuple[int, int, int],
) -> tuple[float, float]:
    """Two lower bounds of the route cost between two grid states (x, y, angle).

    The first is the distance in um; the second adds one bend where the
    heading-aware heuristic sees that the source heading misses the target.
    """
    source_x, source_y, source_angle = source
    target_x, target_y, target_angle = target
    grid_size_um = float(state.obstacles.grid.grid_size_um)
    dx = target_x - source_x
    dy = target_y - source_y
    distance = math.hypot(float(dx), float(dy)) * grid_size_um
    heading_lower_bound = distance
    if str(settings.heuristic_mode) == "heading_aware":
        target_angle_ok = not bool(
            getattr(state.kernel.astar_cfg, "require_target_angle", True)
        ) or (source_angle % 8 == target_angle % 8)
        reaches_target_ray = _direction_reaches_target_ray(
            source=source,
            target_cell=(target_x, target_y),
            tolerance=max(0, int(getattr(state.kernel.astar_cfg, "target_tolerance_cells", 0))),
        )
        if not target_angle_ok or not reaches_target_ray:
            minimum_bend_units = 1.0 if settings.allow_45_degree_turns else 2.0
            bend_weight = float(getattr(state.kernel.astar_cfg, "bend_weight", 1.0))
            heading_lower_bound += minimum_bend_units * bend_weight
    return distance, heading_lower_bound


def _committed_dynamic_cells(
    state: SessionState, *, exclude_net_id: int | None = None
) -> set[tuple[int, int]]:
    return state.search.bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)


def _committed_dynamic_cells_for_attempt(
    state: SessionState,
    *,
    exclude_net_id: int | None = None,
) -> set[tuple[int, int]]:
    if state.search.bookkeeping.diagnostics_enabled:
        return state.search.bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)
    merged: set[tuple[int, int]] = set()
    for net_id in state.search.bookkeeping.records_by_id:
        if exclude_net_id is not None and int(net_id) == int(exclude_net_id):
            continue
        merged.update(route_cells_from_router(state, net_id))
    return merged


def route_cells_from_router(state: SessionState, net_id: int) -> set[tuple[int, int]]:
    """Return the cells the kernel router holds for a net as a set of (x, y) integer pairs."""
    return {(int(cell[0]), int(cell[1])) for cell in state.kernel.router.get_net_cells(int(net_id))}


def state_openings_for_job(
    state: SessionState,
    job: RouteJob,
) -> StateOpenings:
    """Return the states and opened cells precomputed for a job in the static hand-off.

    The tuple has the layout that `states_and_openings` returns. Raises KeyError if
    the job's net has no entry in ``state.run.state_openings_by_id``.
    """
    return state.run.state_openings_by_id[int(job.net_id)]


def _static_cells_in_rect(
    state: SessionState, min_x: int, max_x: int, min_y: int, max_y: int
) -> int:
    if min_x > max_x or min_y > max_y:
        return 0
    if state.static_cells.blocked_rects_for_diagnostics:
        return sum(
            rect_overlap_cell_count(
                rect,
                min_x=min_x,
                max_x=max_x,
                min_y=min_y,
                max_y=max_y,
            )
            for rect in state.static_cells.blocked_rects_for_diagnostics
        )
    return sum(
        1
        for x, y in state.static_cells.blocked_cells_before_port_reservations
        if min_x <= x <= max_x and min_y <= y <= max_y
    )


def _cells_in_rect(
    cells: set[tuple[int, int]],
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    if min_x > max_x or min_y > max_y:
        return 0
    return sum(1 for x, y in cells if min_x <= x <= max_x and min_y <= y <= max_y)


def _raw_dynamic_consistency(
    state: SessionState, span_bbox: tuple[int, int, int, int]
) -> Mapping[str, object]:
    """Counts that compare the kernel's raw dynamic cells with the cells of its nets.

    How many raw dynamic, raw core and net route cells there are, how many
    cells more than one route holds, how many raw dynamic cells belong to
    no net (with bounding box and a sample of twelve), how many net cells
    and core cells are not dynamic, and the same inside ``span_bbox``
    (min x, max x, min y, max y). A set the kernel does not expose counts
    as empty.
    """

    span_bbox_min_x, span_bbox_max_x, span_bbox_min_y, span_bbox_max_y = span_bbox
    raw_net_route_cells: set[tuple[int, int]] = set()
    raw_dynamic_entries = [
        (int(x), int(y), int(refs))
        for x, y, refs in state.kernel.router.raw_dynamic_obstacle_cells()
    ]
    raw_dynamic_cells = {(x, y) for x, y, _ in raw_dynamic_entries}
    raw_dynamic_refcount_gt1_cells = {(x, y) for x, y, refs in raw_dynamic_entries if refs > 1}
    raw_core_entries = [
        (int(x), int(y), int(refs)) for x, y, refs in state.kernel.router.raw_dynamic_core_cells()
    ]
    raw_core_cells = {(x, y) for x, y, _ in raw_core_entries}
    raw_core_refcount_gt1_cells = {(x, y) for x, y, refs in raw_core_entries if refs > 1}
    for _, cells in state.kernel.router.all_net_route_cells():
        raw_net_route_cells.update((int(cell[0]), int(cell[1])) for cell in cells)
    raw_dynamic_without_owner = raw_dynamic_cells - raw_net_route_cells
    raw_net_route_without_dynamic = raw_net_route_cells - raw_dynamic_cells
    raw_core_without_dynamic = raw_core_cells - raw_dynamic_cells
    raw_dynamic_span_cells = {
        (x, y)
        for x, y in raw_dynamic_cells
        if span_bbox_min_x <= x <= span_bbox_max_x and span_bbox_min_y <= y <= span_bbox_max_y
    }
    raw_dynamic_without_owner_span_cells = {
        (x, y)
        for x, y in raw_dynamic_without_owner
        if span_bbox_min_x <= x <= span_bbox_max_x and span_bbox_min_y <= y <= span_bbox_max_y
    }
    return {
        "raw_dynamic_obstacle_cells_before": len(raw_dynamic_cells),
        "raw_dynamic_core_cells_before": len(raw_core_cells),
        "raw_net_route_cells_before": len(raw_net_route_cells),
        "raw_dynamic_refcount_gt1_count": len(raw_dynamic_refcount_gt1_cells),
        "raw_dynamic_refcount_gt1_bbox": cells_bbox(raw_dynamic_refcount_gt1_cells),
        "raw_core_refcount_gt1_count": len(raw_core_refcount_gt1_cells),
        "raw_dynamic_without_owner_count": len(raw_dynamic_without_owner),
        "raw_dynamic_without_owner_bbox": cells_bbox(raw_dynamic_without_owner),
        "raw_dynamic_without_owner_sample": sorted(raw_dynamic_without_owner)[:12],
        "raw_net_route_without_dynamic_count": len(raw_net_route_without_dynamic),
        "raw_net_route_without_dynamic_bbox": cells_bbox(raw_net_route_without_dynamic),
        "raw_core_without_dynamic_count": len(raw_core_without_dynamic),
        "span_raw_dynamic_cells": len(raw_dynamic_span_cells),
        "span_raw_dynamic_without_owner_count": len(raw_dynamic_without_owner_span_cells),
        "span_raw_dynamic_without_owner_bbox": cells_bbox(raw_dynamic_without_owner_span_cells),
    }


def route_attempt_diagnostics(
    settings: SessionSettings,
    state: SessionState,
    job: RouteJob,
    route_obj: object | None,
    *,
    candidate_blockers: list[int] | None = None,
    ripup_ids: list[int] | None = None,
) -> dict[str, object]:
    """Collect the diagnostic figures of one route attempt of a job as a flat dict.

    All sizes are in cells. The dict holds the two grid states and their span, the
    search window and the bounding box of the route with their areas and ratios, the
    static and the committed dynamic cells in the window and in the span box, the
    route cost against the Euclidean and the heading lower bound, consistency counts
    of the kernel's raw dynamic cells against the cells of its nets, and the net ids
    and route indices of the candidate blockers and the rip-up victims.

    ``route_obj`` is None for an attempt without a route; window and route box are
    then empty and the cost is None.
    """
    dynamic_cells_before = _committed_dynamic_cells_for_attempt(state, exclude_net_id=job.net_id)
    source_state, target_state, _, _, opened_cells = state_openings_for_job(state, job)
    source_x = int(source_state.x)
    source_y = int(source_state.y)
    source_angle = int(source_state.angle)
    target_x = int(target_state.x)
    target_y = int(target_state.y)
    target_angle = int(target_state.angle)
    span_x = abs(target_x - source_x)
    span_y = abs(target_y - source_y)
    span_bbox_min_x = min(source_x, target_x)
    span_bbox_max_x = max(source_x, target_x)
    span_bbox_min_y = min(source_y, target_y)
    span_bbox_max_y = max(source_y, target_y)
    span_bbox_area = rect_cell_count(
        min_x=span_bbox_min_x,
        max_x=span_bbox_max_x,
        min_y=span_bbox_min_y,
        max_y=span_bbox_max_y,
    )
    window_min_x = int(getattr(route_obj, "last_window_min_x", 0)) if route_obj else 0
    window_max_x = int(getattr(route_obj, "last_window_max_x", -1)) if route_obj else -1
    window_min_y = int(getattr(route_obj, "last_window_min_y", 0)) if route_obj else 0
    window_max_y = int(getattr(route_obj, "last_window_max_y", -1)) if route_obj else -1
    window_area = int(getattr(route_obj, "last_window_area_cells", 0)) if route_obj else 0
    if window_area <= 0:
        window_area = rect_cell_count(
            min_x=window_min_x,
            max_x=window_max_x,
            min_y=window_min_y,
            max_y=window_max_y,
        )
    window_static_cells = _static_cells_in_rect(
        state,
        window_min_x,
        window_max_x,
        window_min_y,
        window_max_y,
    )
    window_dynamic_cells = _cells_in_rect(
        dynamic_cells_before,
        min_x=window_min_x,
        max_x=window_max_x,
        min_y=window_min_y,
        max_y=window_max_y,
    )
    span_static_cells = _static_cells_in_rect(
        state,
        span_bbox_min_x,
        span_bbox_max_x,
        span_bbox_min_y,
        span_bbox_max_y,
    )
    span_dynamic_cells = _cells_in_rect(
        dynamic_cells_before,
        min_x=span_bbox_min_x,
        max_x=span_bbox_max_x,
        min_y=span_bbox_min_y,
        max_y=span_bbox_max_y,
    )
    route_cells = getattr(route_obj, "cells", None) if route_obj is not None else None
    route_bbox = route_cells_bbox(route_cells or ())
    if route_bbox is None:
        route_bbox_min_x = 0
        route_bbox_max_x = -1
        route_bbox_min_y = 0
        route_bbox_max_y = -1
    else:
        route_bbox_min_x, route_bbox_max_x, route_bbox_min_y, route_bbox_max_y = route_bbox
    route_bbox_area = rect_cell_count(
        min_x=route_bbox_min_x,
        max_x=route_bbox_max_x,
        min_y=route_bbox_min_y,
        max_y=route_bbox_max_y,
    )
    total_cost = float(getattr(route_obj, "total_cost", 0.0)) if route_obj is not None else None
    euclidean_lower_bound, heading_lower_bound = _source_lower_bounds(
        settings,
        state,
        source=(source_x, source_y, source_angle),
        target=(target_x, target_y, target_angle),
    )
    blocker_ids = list(candidate_blockers or [])
    victim_ids = list(ripup_ids or [])
    return {
        "source_state": [source_x, source_y, source_angle],
        "target_state": [target_x, target_y, target_angle],
        "span_x_cells": span_x,
        "span_y_cells": span_y,
        "span_manhattan_cells": span_x + span_y,
        "span_bbox_area_cells": span_bbox_area,
        "span_static_cells": span_static_cells,
        "span_dynamic_cells": span_dynamic_cells,
        "route_bbox_min_x": route_bbox_min_x,
        "route_bbox_max_x": route_bbox_max_x,
        "route_bbox_min_y": route_bbox_min_y,
        "route_bbox_max_y": route_bbox_max_y,
        "route_bbox_width_cells": max(0, route_bbox_max_x - route_bbox_min_x + 1),
        "route_bbox_height_cells": max(0, route_bbox_max_y - route_bbox_min_y + 1),
        "route_bbox_area_cells": route_bbox_area,
        "route_bbox_to_span_bbox_area": (
            float(route_bbox_area) / float(span_bbox_area)
            if span_bbox_area > 0 and route_bbox_area > 0
            else None
        ),
        "opened_cells_count": len(opened_cells),
        "block_radius_cells": state.kernel.radii.block_radius_cells,
        "dynamic_obstacle_search_expansion_radius_cells": (
            state.kernel.radii.clearance_policy.dynamic_obstacle_search_expansion_radius_cells
        ),
        "dynamic_route_commit_keepout_radius_cells": (
            state.kernel.radii.clearance_policy.dynamic_route_commit_keepout_radius_cells
        ),
        "dynamic_route_core_radius_cells": (
            state.kernel.radii.clearance_policy.dynamic_route_core_radius_cells
        ),
        "bend_radius_cells": state.kernel.radii.bend_radius_cells,
        "window_width_cells": max(0, window_max_x - window_min_x + 1),
        "window_height_cells": max(0, window_max_y - window_min_y + 1),
        "window_area_cells": window_area,
        "window_to_span_bbox_area": (
            float(window_area) / float(span_bbox_area)
            if span_bbox_area > 0 and window_area > 0
            else None
        ),
        "route_bbox_to_window_area": (
            float(route_bbox_area) / float(window_area)
            if window_area > 0 and route_bbox_area > 0
            else None
        ),
        "window_static_cells": window_static_cells,
        "window_dynamic_cells": window_dynamic_cells,
        "window_static_density": (
            float(window_static_cells) / float(window_area) if window_area > 0 else None
        ),
        "window_dynamic_density": (
            float(window_dynamic_cells) / float(window_area) if window_area > 0 else None
        ),
        "total_cost": total_cost,
        "euclidean_lower_bound_cost": euclidean_lower_bound,
        "heading_lower_bound_cost": heading_lower_bound,
        "euclidean_lower_bound_to_cost": (
            euclidean_lower_bound / total_cost
            if total_cost is not None and total_cost > 0.0
            else None
        ),
        "heading_lower_bound_to_cost": (
            heading_lower_bound / total_cost
            if total_cost is not None and total_cost > 0.0
            else None
        ),
        "heading_lower_bound_gap_cost": (
            total_cost - heading_lower_bound if total_cost is not None else None
        ),
        "committed_dynamic_cells_before": len(dynamic_cells_before),
        **_raw_dynamic_consistency(
            state, (span_bbox_min_x, span_bbox_max_x, span_bbox_min_y, span_bbox_max_y)
        ),
        "candidate_blocker_count": len(blocker_ids),
        "candidate_blocker_net_ids": blocker_ids,
        "candidate_blocker_route_indices": [
            state.run.route_jobs_by_id[net_id].route_index
            for net_id in blocker_ids
            if net_id in state.run.route_jobs_by_id
        ],
        "ripup_victim_count": len(victim_ids),
        "ripup_victim_net_ids": victim_ids,
        "ripup_victim_route_indices": [
            state.run.route_jobs_by_id[net_id].route_index
            for net_id in victim_ids
            if net_id in state.run.route_jobs_by_id
        ],
    }


def export_route_svg(
    settings: SessionSettings,
    state: SessionState,
    job: RouteJob,
    route_obj: Any,
    *,
    suffix: str = "",
    obstacle_cells: set[tuple[int, int]] | None = None,
    opened_cells: list[tuple[int, int]] | None = None,
) -> None:
    """Write the debug SVG of one route and add its path to ``state.search.route_svgs``.

    The file is ``<debug_path>/routes/<debug_prefix>_<net_name><suffix>.svg``. On top of
    the kernel's drawing it marks in red the port keep-out cells that are not opened
    for the job, and labels the net with its routed length in um. ``obstacle_cells``
    replaces the obstacles the kernel draws, if the kernel supports that.
    ``opened_cells`` defaults to the job's precomputed opened cells. Does nothing
    without a debug path or if ``settings.debug_route_indices`` excludes the job.
    """
    debug_path = state.obstacles.debug_path
    selected = (
        settings.debug_route_indices is None or job.route_index in settings.debug_route_indices
    )
    if debug_path is None or not selected:
        return
    route_dir = debug_path / "routes"
    ensure_dir(route_dir)
    route_svg = route_dir / f"{settings.debug_prefix}_{job.net_name}{suffix}.svg"
    if obstacle_cells is not None:
        svg_text = state.kernel.router.export_debug_svg_with_obstacle_cells(
            route_obj,
            sorted(obstacle_cells),
        )
    else:
        svg_text = state.kernel.router.export_debug_svg(route_obj)
    if opened_cells is None:
        try:
            _, _, _, _, opened_cells = state_openings_for_job(state, job)
        except (KeyError, TypeError):
            # a job the run stored no openings for
            opened_cells = []
    if settings.debug_stop_after_route_index is not None and int(job.route_index) == int(
        settings.debug_stop_after_route_index
    ):
        next_job = state.run.full_route_jobs_by_route_index.get(
            int(settings.debug_stop_after_route_index) + 1
        )
        if next_job is not None:
            with contextlib.suppress(Exception):
                _, _, _, _, opened_cells = states_and_openings(settings, state, next_job)
    red_keepout_cells = state.static_cells.debug_port_keepout_cells - {
        (int(cell[0]), int(cell[1])) for cell in opened_cells
    }
    if red_keepout_cells:
        overlay = ['<g id="port-keepout-cells">']
        for gx, gy in sorted(red_keepout_cells):
            if 0 <= gx < state.kernel.frame.grid_width and 0 <= gy < state.kernel.frame.grid_height:
                svg_y = state.kernel.frame.grid_height - gy - 1
                overlay.append(
                    f'<rect class="port-keepout" x="{gx}" y="{svg_y}" '
                    'width="1" height="1" fill="#d93025" opacity="0.38" />'
                )
        overlay.append("</g>")
        overlay_text = "".join(overlay)
        if "</svg>" in svg_text and 'id="port-keepout-cells"' not in svg_text:
            svg_text = svg_text.replace("</svg>", overlay_text + "</svg>", 1)
    # The route's name and routed length (route-overview plan, 2026-10-05;
    # before meanders: the path-length matching runs after dispatch, the
    # realized lengths are in the routes overview SVG and the meander report).
    routed_length_um = float(getattr(route_obj, "total_length_um", 0.0) or 0.0)
    font_size = max(1.0, float(state.kernel.frame.grid_height) * 0.025)
    label = (
        f'<text id="route-length-label" x="{font_size * 0.5:.2f}" y="{font_size * 1.2:.2f}" '
        f'font-size="{font_size:.2f}" font-family="sans-serif" fill="#111827" '
        f'paint-order="stroke" stroke="white" stroke-width="{font_size * 0.2:.2f}">'
        f"{_xml_escape(job.net_name)}: routed {routed_length_um:.1f} um (before meanders)"
        "</text>"
    )
    if "</svg>" in svg_text and 'id="route-length-label"' not in svg_text:
        svg_text = svg_text.replace("</svg>", label + "</svg>", 1)
    route_svg.write_text(svg_text, encoding="utf-8")
    state.search.route_svgs.append(route_svg)


def route_engine_summary(route_obj: Any) -> str:
    """Summarize a route object in one line: kind, length in um, cost and expanded states.

    The kind is ``simple`` if the route was found without expanding a search state,
    ``astar`` otherwise.
    """
    expanded_states = int(getattr(route_obj, "expanded_states", 0))
    route_kind = "simple" if expanded_states == 0 else "astar"
    length_um = as_float(getattr(route_obj, "total_length_um", 0.0), 0.0)
    total_cost = as_float(getattr(route_obj, "total_cost", 0.0), 0.0)
    return f"{route_kind} length={length_um:.3f}um cost={total_cost:.3f} expanded={expanded_states}"


_EIGHT_NEIGHBORS = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


def _cells_inflated_by(cells: set[tuple[int, int]], radius: int) -> set[tuple[int, int]]:
    """Return ``cells`` grown by ``radius`` cells in every direction (a square), as a new set."""

    if radius <= 0:
        return set(cells)
    inflated: set[tuple[int, int]] = set()
    for x, y in cells:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                inflated.add((x + dx, y + dy))
    return inflated


def _cells_reachable_from(
    start: tuple[int, int],
    blocked: set[tuple[int, int]],
    grid_size: tuple[int, int],
) -> set[tuple[int, int]]:
    """Return the cells reachable from ``start`` over the eight neighbours.

    Only cells inside the ``grid_size`` (width, height) grid and outside
    ``blocked`` are entered; a start outside the grid or blocked gives the
    empty set.
    """

    grid_width, grid_height = grid_size
    sx, sy = start
    if not (0 <= sx < grid_width and 0 <= sy < grid_height):
        return set()
    if start in blocked:
        return set()
    reached = {start}
    queue: deque[tuple[int, int]] = deque([start])
    while queue:
        x, y = queue.popleft()
        for dx, dy in _EIGHT_NEIGHBORS:
            nx = x + dx
            ny = y + dy
            neighbor = (nx, ny)
            if (
                0 <= nx < grid_width
                and 0 <= ny < grid_height
                and neighbor not in blocked
                and neighbor not in reached
            ):
                reached.add(neighbor)
                queue.append(neighbor)
    return reached


def _corridor_clearance_diagnostic(
    state: SessionState,
    source_state: Any,
    target_state: Any,
    blocked_cells: set[tuple[int, int]],
    *,
    max_radius: int | None = None,
) -> dict[str, Any]:
    max_radius = (
        max(4, int(state.kernel.radii.bend_radius_cells) + 2)
        if max_radius is None
        else int(max_radius)
    )

    source = (int(source_state.x), int(source_state.y))
    target = (int(target_state.x), int(target_state.y))
    grid_size = (state.kernel.frame.grid_width, state.kernel.frame.grid_height)

    def inflate(radius: int) -> set[tuple[int, int]]:
        inflated = _cells_inflated_by(blocked_cells, radius)
        inflated.discard(source)
        inflated.discard(target)
        return inflated

    def reachable_from(
        start: tuple[int, int],
        blocked: set[tuple[int, int]],
    ) -> set[tuple[int, int]]:
        return _cells_reachable_from(start, blocked, grid_size)

    last_connected_radius: int | None = None
    first_disconnected_radius: int | None = None
    disconnected_blocked: set[tuple[int, int]] | None = None
    for radius in range(max_radius + 1):
        inflated = inflate(radius)
        reachable = reachable_from(source, inflated)
        if target in reachable:
            last_connected_radius = radius
            continue
        first_disconnected_radius = radius
        disconnected_blocked = inflated
        break

    source_region_size: int | None = None
    target_region_size: int | None = None
    source_region_min_distance_to_target: int | None = None
    target_region_min_distance_to_source: int | None = None
    if first_disconnected_radius is not None and disconnected_blocked is not None:
        source_region = reachable_from(source, disconnected_blocked)
        target_region = reachable_from(target, disconnected_blocked)
        source_region_size = len(source_region)
        target_region_size = len(target_region)
        if target in source_region:
            source_region_min_distance_to_target = 0
        elif source_region:
            source_region_min_distance_to_target = min(
                abs(x - target[0]) + abs(y - target[1]) for x, y in source_region
            )
        if source in target_region:
            target_region_min_distance_to_source = 0
        elif target_region:
            target_region_min_distance_to_source = min(
                abs(x - source[0]) + abs(y - source[1]) for x, y in target_region
            )

    return {
        "max_radius_checked": max_radius,
        "last_connected_radius": last_connected_radius,
        "first_disconnected_radius": first_disconnected_radius,
        "source_region_size": source_region_size,
        "target_region_size": target_region_size,
        "source_region_min_distance_to_target": source_region_min_distance_to_target,
        "target_region_min_distance_to_source": target_region_min_distance_to_source,
    }


def write_failed_log(
    settings: SessionSettings,
    state: SessionState,
    job: RouteJob,
    openings: StateOpenings,
    error_text: str,
) -> None:
    """Write ``<debug_prefix>_<net_name>_FAILED.txt`` for a net that could not be routed.

    The file goes to ``<debug_path>/routes`` and has one ``key=value`` line per item:
    the two port specs and grid states, the radii and lane sizes in cells, the overlap
    of the opened cells with static and committed dynamic cells, the corridor
    clearance diagnostic, the error, the illegal-crossing root causes and the repair
    trace if any, and the last attempts of this net and of the run. Does nothing
    without a debug path.
    """
    if state.obstacles.debug_path is None:
        return
    source_state = openings.source_state
    target_state = openings.target_state
    opened_candidate_cells = openings.opened_candidate_cells
    opened_cells = openings.opened_cells
    route_dir = state.obstacles.debug_path / "routes"
    ensure_dir(route_dir)
    port1_spec = f"{job.inst1},{job.port1}"
    port2_spec = f"{job.inst2},{job.port2}"
    fail_txt = route_dir / f"{settings.debug_prefix}_{job.net_name}_FAILED.txt"
    committed_dynamic_cells = _committed_dynamic_cells(state)
    opened_candidate_static_overlap = (
        opened_candidate_cells & state.static_cells.blocked_cells_before_port_reservations
    )
    opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
    opened_cells_set = set(opened_cells)
    corridor_diagnostic = _corridor_clearance_diagnostic(
        state,
        source_state,
        target_state,
        (state.static_cells.blocked_cells_before_port_reservations - opened_cells_set)
        | committed_dynamic_cells,
    )
    opened_static_overlap = (
        opened_cells_set & state.static_cells.blocked_cells_before_port_reservations
    )
    opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
    current_attempts = [
        ("current", record)
        for record in state.search.attempt_records
        if getattr(record, "net_id", None) == job.net_id
    ]
    recent_attempts = [("recent", record) for record in state.search.attempt_records[-12:]]
    root_cause_line = format_illegal_crossing_root_causes_line(
        [error_text]
        + [
            str(attempt.error)
            for _, attempt in (current_attempts[-8:] + recent_attempts)
            if getattr(attempt, "error", None)
        ]
    )
    fail_lines = [
        f"net_name={job.net_name}",
        f"source_spec={port1_spec}",
        f"target_spec={port2_spec}",
        f"source_state=({int(source_state.x)}, {int(source_state.y)}, {int(source_state.angle)})",
        f"target_state=({int(target_state.x)}, {int(target_state.y)}, {int(target_state.angle)})",
        f"allow_45_degree_turns={settings.allow_45_degree_turns}",
        f"block_radius_cells={state.kernel.radii.block_radius_cells}",
        (
            "dynamic_obstacle_search_expansion_radius_cells="
            f"{state.kernel.radii.clearance_policy.dynamic_obstacle_search_expansion_radius_cells}"
        ),
        (
            "dynamic_route_commit_keepout_radius_cells="
            f"{state.kernel.radii.clearance_policy.dynamic_route_commit_keepout_radius_cells}"
        ),
        f"dynamic_route_core_radius_cells={state.kernel.radii.clearance_policy.dynamic_route_core_radius_cells}",
        f"bend_radius_cells={state.kernel.radii.bend_radius_cells}",
        f"port_lane_length_cells={state.kernel.lanes.port_lane_length_cells}",
        f"port_lane_half_width_cells={state.kernel.lanes.port_lane_half_width_cells}",
        f"opened_candidate_cells_count={len(opened_candidate_cells)}",
        f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
        f"opened_candidate_static_overlap_bbox={cells_bbox(opened_candidate_static_overlap)}",
        f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
        f"opened_candidate_dynamic_overlap_bbox={cells_bbox(opened_candidate_dynamic_overlap)}",
        f"opened_cells_count={len(opened_cells)}",
        f"opened_static_overlap_count={len(opened_static_overlap)}",
        f"opened_static_overlap_bbox={cells_bbox(opened_static_overlap)}",
        f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
        f"opened_dynamic_overlap_bbox={cells_bbox(opened_dynamic_overlap)}",
        f"corridor_clearance_max_radius_checked={corridor_diagnostic['max_radius_checked']}",
        f"corridor_clearance_last_connected_radius={corridor_diagnostic['last_connected_radius']}",
        f"corridor_clearance_first_disconnected_radius={corridor_diagnostic['first_disconnected_radius']}",
        f"corridor_clearance_source_region_size={corridor_diagnostic['source_region_size']}",
        f"corridor_clearance_target_region_size={corridor_diagnostic['target_region_size']}",
        (
            "corridor_clearance_source_region_min_distance_to_target="
            f"{corridor_diagnostic['source_region_min_distance_to_target']}"
        ),
        (
            "corridor_clearance_target_region_min_distance_to_source="
            f"{corridor_diagnostic['target_region_min_distance_to_source']}"
        ),
        f"error={error_text}",
    ]
    if root_cause_line is not None:
        fail_lines.append(root_cause_line)
    fail_lines.extend(format_native_repair_trace_lines(state.search.native_repair_trace))
    for label, attempt in current_attempts[-8:] + recent_attempts:
        as_dict = attempt.as_dict()
        diagnostics = as_dict.get("diagnostics")
        fail_lines.append(
            f"attempt_{label}="
            + ", ".join(
                f"{key}={as_dict.get(key)}"
                for key in (
                    "attempt_index",
                    "bucket_name",
                    "failed",
                    "error",
                    "elapsed_s",
                    "expanded_states",
                    "generated_neighbors",
                    "window_attempts",
                    "used_full_grid_fallback",
                    "candidate_blocker_count",
                    "candidate_blocker_route_indices",
                    "ripup_victim_count",
                    "ripup_victim_route_indices",
                )
                if key in as_dict
            )
        )
        if isinstance(diagnostics, dict) and diagnostics:
            fail_lines.append(
                f"attempt_{label}_diagnostics="
                + ", ".join(
                    f"{key}={diagnostics.get(key)}"
                    for key in (
                        "candidate_blocker_count",
                        "candidate_blocker_route_indices",
                        "ripup_victim_count",
                        "ripup_victim_route_indices",
                        "route_bbox_min_x",
                        "route_bbox_max_x",
                        "route_bbox_min_y",
                        "route_bbox_max_y",
                    )
                    if key in diagnostics
                )
            )
    fail_txt.write_text("\n".join(fail_lines) + "\n", encoding="utf-8")
