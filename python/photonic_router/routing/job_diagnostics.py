"""Phase 6 diagnostics: the text file of one routed or failed job.

`write_route_diagnostics` writes one ``key=value`` line per item. The lines
come in this order, each group from its own function:

1. the job, its ports and its two grid states (`_job_lines`)
2. the cells the job opened and what they overlap (`_opened_lines`)
3. the runways of the job's ports and its clearance-exempt cells
   (`_runway_and_exempt_lines`)
4. what the cells of the route overlap (`_route_lines`)
5. for a route with one crossing between two horizontal ports: the
   90-degree paths behind the crossing (`_post_crossing_candidate_lines`)
6. the four bends at the target port (`_target_bend_footprint_lines`)
7. the first moves from the source port (`_first_move_lines`)
8. the route's segments and the error text, where there are any

Nothing in here takes a routing decision. `dispatch.py` hands a file path
only when the run has a debug directory, so the committed cells of the
other nets are always tracked when a file is written.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from photonic_router.gds.route_rust_debug_artifacts import cells_bbox
from photonic_router.records.route_rust_obstacle_config import schematic_instance_component_name
from photonic_router.records.route_rust_types import RouteJob
from photonic_router.routing.crossing_plan_stage import cells_in_raw_static_geometry
from photonic_router.routing.diagnostics import route_cells_from_router
from photonic_router.routing.handoff import foreign_keepout_open_cells_for_spec
from photonic_router.routing.products import StateOpenings
from photonic_router.routing.route_jobs import angle_to_step, fanout_stub_bend_steps
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.state import SessionState

_Cell = tuple[int, int]


@dataclass(frozen=True)
class _JobCells:
    """The cells of one job that the write-up compares, all in grid cells."""

    net_id: int
    source: _Cell
    source_angle: int
    target: _Cell
    target_angle: int
    #: the cells the job may open, and the ones it opens
    opened_candidate: set[_Cell]
    opened: set[_Cell]
    #: the cells of the route; empty for a failed job
    route: set[_Cell]
    #: the cells the other nets hold
    committed: set[_Cell]
    #: the cells where the job need not keep its clearance to other nets
    exempt: set[_Cell]

    @property
    def route_on_committed(self) -> set[_Cell]:
        """The cells of the route that another net holds too."""
        return self.route & self.committed


def write_route_diagnostics(
    settings: SessionSettings,
    state: SessionState,
    *,
    job: RouteJob,
    openings: StateOpenings,
    diag_txt: Path | None,
    route_obj: Any | None = None,
    error_text: str | None = None,
) -> None:
    """Write the diagnostics text file of one routed or failed job to ``diag_txt``.

    The file has one ``key=value`` line per item, in the order of the
    module docstring; the status is ``failed`` if ``error_text`` is given,
    otherwise ``ok``. Does nothing if ``diag_txt`` is None.
    """
    if diag_txt is None:
        return
    cells = _job_cells(state, job, openings, route_obj)
    lines = [
        *_job_lines(settings, state, job, cells, failed=error_text is not None),
        *_opened_lines(state, cells),
        *_runway_and_exempt_lines(state, job, cells),
        *_route_lines(state, job, cells),
        *_post_crossing_candidate_lines(state, cells),
        *_target_bend_footprint_lines(state, cells),
        *_first_move_lines(settings, state, cells),
    ]
    route_segments = _route_segment_texts(route_obj)
    if route_segments:
        lines.append("route_segments=" + "; ".join(route_segments))
    if error_text is not None:
        lines.append(f"error={error_text}")
    diag_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _job_cells(
    state: SessionState, job: RouteJob, openings: StateOpenings, route_obj: Any | None
) -> _JobCells:
    return _JobCells(
        net_id=int(job.net_id),
        source=(int(openings.source_state.x), int(openings.source_state.y)),
        source_angle=int(openings.source_state.angle),
        target=(int(openings.target_state.x), int(openings.target_state.y)),
        target_angle=int(openings.target_state.angle),
        opened_candidate=openings.opened_candidate_cells,
        opened=openings.opened_cells_set,
        route={(int(cell[0]), int(cell[1])) for cell in (getattr(route_obj, "cells", None) or [])},
        committed=state.search.bookkeeping.committed_dynamic_cells(exclude_net_id=job.net_id),
        exempt=set(state.run.clearance_exempt_cells_by_id.get(int(job.net_id), [])),
    )


def _port_specs(job: RouteJob) -> tuple[str, str]:
    return f"{job.inst1},{job.port1}", f"{job.inst2},{job.port2}"


def _job_lines(
    settings: SessionSettings,
    state: SessionState,
    job: RouteJob,
    cells: _JobCells,
    *,
    failed: bool,
) -> list[str]:
    source_spec, target_spec = _port_specs(job)
    foreign_keepout_cells = set(foreign_keepout_open_cells_for_spec(state, source_spec))
    foreign_keepout_cells.update(foreign_keepout_open_cells_for_spec(state, target_spec))
    cluster_specs = state.runways.source_cluster_specs_by_port_spec
    return [
        f"net_name={job.net_name}",
        f"status={'failed' if failed else 'ok'}",
        f"source_spec={source_spec}",
        f"target_spec={target_spec}",
        f"source_component={schematic_instance_component_name(settings.schematic, job.inst1)}",
        f"target_component={schematic_instance_component_name(settings.schematic, job.inst2)}",
        f"source_access_rule={state.ports.access_rule_by_spec.get(source_spec)}",
        f"target_access_rule={state.ports.access_rule_by_spec.get(target_spec)}",
        f"foreign_port_keepout_cells={int(settings.foreign_port_keepout_cells)}",
        f"fanout_access_mode={settings.fanout_access_mode_normalized}",
        f"fanout_stub_bend_degrees={45 * int(fanout_stub_bend_steps(settings))}",
        f"fanout_anchor_port_count={len(state.fanout.anchor_by_port_spec)}",
        f"fanout_stub_center_cell_count={len(state.fanout.stub_center_cells)}",
        f"fanout_stub_static_cell_count={len(state.fanout.stub_static_cells)}",
        f"source_fanout_anchor={source_spec in state.fanout.anchor_by_port_spec}",
        f"target_fanout_anchor={target_spec in state.fanout.anchor_by_port_spec}",
        f"source_dense_port_runway_cells={state.runways.source_length_by_spec.get(source_spec)}",
        f"target_dense_port_runway_cells={state.runways.target_length_by_spec.get(target_spec)}",
        f"source_dense_source_cluster_size={len(cluster_specs.get(source_spec, set()))}",
        f"target_dense_source_cluster_size={len(cluster_specs.get(target_spec, set()))}",
        f"foreign_port_keepout_static_count={len(state.static_cells.foreign_keepout_cells)}",
        f"foreign_port_keepout_open_count={len(foreign_keepout_cells & cells.opened)}",
        f"source_state=({cells.source[0]}, {cells.source[1]}, {cells.source_angle})",
        f"target_state=({cells.target[0]}, {cells.target[1]}, {cells.target_angle})",
    ]


def _opened_lines(state: SessionState, cells: _JobCells) -> list[str]:
    candidate_static = cells_in_raw_static_geometry(state, cells.opened_candidate)
    candidate_committed = cells.opened_candidate & cells.committed
    opened_static = cells_in_raw_static_geometry(state, cells.opened)
    opened_committed = cells.opened & cells.committed
    return [
        f"opened_candidate_cells_count={len(cells.opened_candidate)}",
        f"opened_candidate_static_overlap_count={len(candidate_static)}",
        f"opened_candidate_static_overlap_bbox={cells_bbox(candidate_static)}",
        f"opened_candidate_dynamic_overlap_count={len(candidate_committed)}",
        f"opened_candidate_dynamic_overlap_bbox={cells_bbox(candidate_committed)}",
        f"opened_cells_count={len(cells.opened)}",
        f"opened_cells={sorted(cells.opened)}",
        f"opened_static_overlap_count={len(opened_static)}",
        f"opened_static_overlap_bbox={cells_bbox(opened_static)}",
        f"opened_dynamic_overlap_count={len(opened_committed)}",
        f"opened_dynamic_overlap_bbox={cells_bbox(opened_committed)}",
        f"opened_dynamic_overlap_owners={_owners_of(state, cells, opened_committed)}",
    ]


def _own_runway_cells(state: SessionState, job: RouteJob) -> set[_Cell]:
    """The runway cells of the job's two ports."""
    runways = state.ports.runway_cells_by_spec
    source_spec, target_spec = _port_specs(job)
    return set(runways.get(source_spec, set())) | set(runways.get(target_spec, set()))


def _sibling_runway_cells(state: SessionState, job: RouteJob) -> set[_Cell]:
    """The runway cells of the other ports in the clusters of the job's two ports."""
    siblings: set[_Cell] = set()
    for port_spec in _port_specs(job):
        for other in state.runways.source_cluster_specs_by_port_spec.get(port_spec, set()):
            if other != port_spec:
                siblings.update(state.ports.runway_cells_by_spec.get(other, set()))
    return siblings


def _runway_and_exempt_lines(state: SessionState, job: RouteJob, cells: _JobCells) -> list[str]:
    own_committed = _own_runway_cells(state, job) & cells.committed
    sibling_committed = _sibling_runway_cells(state, job) & cells.committed
    exempt_committed = cells.exempt & cells.committed
    return [
        f"current_port_runway_dynamic_overlap_count={len(own_committed)}",
        f"current_port_runway_dynamic_overlap_bbox={cells_bbox(own_committed)}",
        f"sibling_port_runway_dynamic_overlap_count={len(sibling_committed)}",
        f"sibling_port_runway_dynamic_overlap_bbox={cells_bbox(sibling_committed)}",
        f"dynamic_clearance_exempt_cells_count={len(cells.exempt)}",
        f"dynamic_clearance_exempt_cells_bbox={cells_bbox(cells.exempt)}",
        f"dynamic_clearance_exempt_dynamic_overlap_count={len(exempt_committed)}",
        f"dynamic_clearance_exempt_dynamic_overlap_bbox={cells_bbox(exempt_committed)}",
    ]


def _route_lines(state: SessionState, job: RouteJob, cells: _JobCells) -> list[str]:
    route = cells.route
    on_static = cells_in_raw_static_geometry(state, route)
    on_committed = cells.route_on_committed
    on_own_runway = route & _own_runway_cells(state, job)
    on_sibling_runway = route & _sibling_runway_cells(state, job)
    candidate_static = route & cells_in_raw_static_geometry(state, cells.opened_candidate)
    opened_static = route & cells_in_raw_static_geometry(state, cells.opened)
    candidate_committed = route & cells.opened_candidate & cells.committed
    opened_committed = route & cells.opened & cells.committed
    return [
        f"route_cells_count={len(route)}",
        f"route_static_blocked_overlap_count={len(on_static)}",
        f"route_static_blocked_overlap_bbox={cells_bbox(on_static)}",
        f"route_dynamic_overlap_count={len(on_committed)}",
        f"route_dynamic_overlap_bbox={cells_bbox(on_committed)}",
        f"route_dynamic_overlap_owners={_owners_of(state, cells, on_committed)}",
        f"route_overlap_current_port_runway_count={len(on_own_runway)}",
        f"route_overlap_current_port_runway_bbox={cells_bbox(on_own_runway)}",
        f"route_overlap_sibling_port_runway_count={len(on_sibling_runway)}",
        f"route_overlap_sibling_port_runway_bbox={cells_bbox(on_sibling_runway)}",
        f"route_overlap_candidate_opened_static_count={len(candidate_static)}",
        f"route_overlap_effective_opened_static_count={len(opened_static)}",
        f"route_overlap_candidate_opened_dynamic_count={len(candidate_committed)}",
        f"route_overlap_effective_opened_dynamic_count={len(opened_committed)}",
        f"route_overlap_dynamic_clearance_exempt_count={len(route & cells.exempt)}",
    ]


def _owners_of(state: SessionState, cells: _JobCells, wanted: set[_Cell]) -> dict[int, list[_Cell]]:
    """Per other net that holds some of `wanted`: those cells, sorted."""
    owners: dict[int, list[_Cell]] = {}
    for net_id in state.search.bookkeeping.records_by_id:
        if int(net_id) == cells.net_id:
            continue
        overlap = wanted & route_cells_from_router(state, int(net_id))
        if overlap:
            owners[int(net_id)] = sorted(overlap)
    return owners


def _search_static_cells(state: SessionState) -> set[_Cell]:
    """The cells the search treats as static: obstacles, port keep-outs and fan-out stubs."""
    static = set(state.static_cells.blocked_cells_before_port_reservations)
    static.update(state.static_cells.debug_port_keepout_cells)
    static.update(state.static_cells.foreign_keepout_cells)
    static.update(state.fanout.stub_static_cells)
    return static


def _post_crossing_candidate_lines(state: SessionState, cells: _JobCells) -> list[str]:
    """For a route with one crossing between two ports facing east at different heights.

    One line per column where a path could leave the crossing's row: the
    path runs on along the row, bends by 90 degrees towards the target's
    row, and bends back into it. The line says what blocks that path.
    """
    crossing_cells = sorted(cells.route_on_committed)
    if (
        len(crossing_cells) != 1
        or cells.source_angle != 0
        or cells.target_angle != 0
        or cells.source[1] == cells.target[1]
        or cells.target[0] <= crossing_cells[0][0]
    ):
        return []
    cross_x, cross_y = crossing_cells[0]
    dy_sign = 1 if cells.target[1] > cross_y else -1
    radius = max(1, int(state.kernel.radii.bend_radius_cells))
    if cells.target[1] - dy_sign * radius == cross_y:
        return []
    static = _search_static_cells(state)
    lines: list[str] = []
    for bend_start_x in range(cross_x + radius, cells.target[0] - 2 * radius + 1):
        path = _post_crossing_path(cells, bend_start_x + radius, cross_y, dy_sign, radius)
        static_blockers = (path & static) - cells.opened
        dynamic_blockers = (
            (path & cells.committed) - cells.route_on_committed - (path & cells.opened)
        )
        lines.append(
            "post_crossing_90_candidate="
            f"bend_start_x={bend_start_x}; "
            f"cells={len(path)}; "
            f"static_blockers={sorted(static_blockers)[:24]}; "
            f"static_blocker_count={len(static_blockers)}; "
            f"dynamic_blockers={sorted(dynamic_blockers)[:24]}; "
            f"dynamic_blocker_count={len(dynamic_blockers)}; "
            f"dynamic_blocker_owners={_owners_of(state, cells, dynamic_blockers)}"
        )
    return lines


def _post_crossing_path(
    cells: _JobCells, corner_x: int, cross_y: int, dy_sign: int, radius: int
) -> set[_Cell]:
    """Along the crossing's row to `corner_x`, up or down that column, then along the target's row."""
    path = {(x, cross_y) for x in range(cells.source[0], corner_x + 1)}
    path.update((corner_x, cross_y + dy_sign * step) for step in range(radius + 1))
    second_start_y = cells.target[1] - dy_sign * radius
    y0, y1 = sorted((cross_y + dy_sign * radius, second_start_y))
    path.update((corner_x, y) for y in range(y0, y1 + 1))
    path.update((corner_x + step, second_start_y + dy_sign * step) for step in range(radius + 1))
    path.update((x, cells.target[1]) for x in range(corner_x + radius, cells.target[0] + 1))
    return path


def _turn_cells(start: _Cell, start_angle: int, delta: int, radius: int) -> list[_Cell]:
    """`radius` cells ahead from `start`, then `radius` cells in the turned direction."""
    start_dir = angle_to_step(start_angle)
    end_dir = angle_to_step((int(start_angle) + int(delta)) % 8)
    corner = (start_dir[0] * radius, start_dir[1] * radius)
    relative = _unique_cells(
        [
            *_line_cells(start=(0, 0), direction=start_dir, cells=radius),
            *_line_cells(start=corner, direction=end_dir, cells=radius),
        ]
    )
    return [(int(start[0]) + dx, int(start[1]) + dy) for dx, dy in relative]


def _target_bend_footprint_lines(state: SessionState, cells: _JobCells) -> list[str]:
    """The four 90-degree bends at the target port and what blocks each."""
    radius = max(1, int(state.kernel.radii.bend_radius_cells))
    static = _search_static_cells(state)
    target = cells.target
    bends: list[tuple[str, _Cell, int, int]] = [
        ("end_from_below_to_port", (target[0] - radius, target[1] - radius), 2, -2),
        ("end_from_above_to_port", (target[0] - radius, target[1] + radius), 6, 2),
        ("target_out_down", target, 0, -2),
        ("target_out_up", target, 0, 2),
    ]
    lines: list[str] = []
    for label, start, angle, delta in bends:
        footprint = set(_turn_cells(start, angle, delta, radius))
        static_blockers = (footprint & static) - cells.opened
        dynamic_blockers = (
            (footprint & cells.committed) - (footprint & cells.opened) - cells.route_on_committed
        )
        lines.append(
            "target_bend_footprint="
            f"label={label}; start={start}; angle={angle}; delta={delta}; "
            f"cells={sorted(footprint)}; "
            f"static_blockers={sorted(static_blockers)}; "
            f"static_blocker_count={len(static_blockers)}; "
            f"dynamic_blockers={sorted(dynamic_blockers)}; "
            f"dynamic_blocker_count={len(dynamic_blockers)}; "
            f"dynamic_blocker_owners={_owners_of(state, cells, dynamic_blockers)}"
        )
    return lines


def _line_cells(*, start: _Cell, direction: _Cell, cells: int) -> list[_Cell]:
    return [
        (int(start[0]) + int(direction[0]) * step, int(start[1]) + int(direction[1]) * step)
        for step in range(max(0, int(cells)) + 1)
    ]


def _unique_cells(cells: Iterable[_Cell]) -> list[_Cell]:
    """`cells` without repeats, in their order."""
    return list(dict.fromkeys((int(cell[0]), int(cell[1])) for cell in cells))


def _first_move_footprint(
    cells: _JobCells, *, kind: str, length: int, delta: int, radius: int
) -> tuple[_Cell, int, list[_Cell]]:
    """The end cell, the end angle and the cells of one first move from the source."""
    if kind == "straight":
        footprint = _line_cells(
            start=cells.source, direction=angle_to_step(cells.source_angle), cells=length
        )
        return footprint[-1], cells.source_angle % 8, footprint
    footprint = _turn_cells(cells.source, cells.source_angle, delta, radius)
    return footprint[-1], (cells.source_angle + int(delta)) % 8, footprint


def _first_move_lines(
    settings: SessionSettings, state: SessionState, cells: _JobCells
) -> list[str]:
    """One line per primitive the search may start with at the source: what it would cross."""
    opened_search = {
        cell
        for cell in cells.opened
        if cell in (cells.source, cells.target) or cell not in cells.committed
    }
    static = _search_static_cells(state)
    primitive_cfg = state.kernel.primitive_cfg
    moves: list[tuple[str, str, int, int]] = [
        ("straight_short", "straight", int(primitive_cfg.straight_short_cells), 0),
        ("straight_long", "straight", int(primitive_cfg.straight_long_cells), 0),
        ("turn45_left", "turn", 0, 1),
        ("turn45_right", "turn", 0, -1),
        ("turn90_left", "turn", 0, 2),
        ("turn90_right", "turn", 0, -2),
    ]
    radius = max(0, int(state.kernel.radii.bend_radius_cells))
    lines: list[str] = []
    for label, kind, length, delta in moves:
        if not settings.allow_45_degree_turns and abs(delta) == 1:
            continue
        end_cell, end_angle, footprint = _first_move_footprint(
            cells, kind=kind, length=length, delta=delta, radius=radius
        )
        covered = set(footprint)
        on_search_static = covered & static
        on_committed = covered & cells.committed
        dynamic_blockers = on_committed - cells.exempt - (covered & opened_search)
        lines.append(
            f"first_move_{label}="
            f"end=({end_cell[0]},{end_cell[1]},{end_angle}); "
            f"footprint={footprint}; "
            f"static={sorted(cells_in_raw_static_geometry(state, covered))}; "
            f"routing_static={sorted(on_search_static)}; "
            f"static_blockers={sorted(on_search_static - opened_search)}; "
            f"dynamic={sorted(on_committed)}; "
            f"dynamic_blockers={sorted(dynamic_blockers)}; "
            f"dynamic_owners={_owners_of(state, cells, on_committed)}; "
            f"opened={sorted(covered & cells.opened)}; "
            f"opened_search={sorted(covered & opened_search)}; "
            f"dynamic_exempt={sorted(covered & cells.exempt)}"
        )
    return lines


def _route_segment_texts(route_obj: Any | None) -> list[str]:
    """One text per segment of the route: its kind, its two ends and its two angles."""
    texts: list[str] = []
    for segment in cast(list[object], getattr(route_obj, "segments", []) or []):
        try:
            entry = dict(cast(Any, segment))
        except (TypeError, ValueError):
            continue
        texts.append(
            "{kind}:{start}->{end}@{start_angle}->{end_angle}".format(
                kind=entry.get("kind"),
                start=entry.get("start"),
                end=entry.get("end"),
                start_angle=entry.get("start_angle"),
                end_angle=entry.get("end_angle"),
            )
        )
    return texts
