"""Stage "route jobs": what phases 3 to 5 hand to the kernel, before any search.

`check_route_jobs` reads the session state after the static hand-off
(`RunPlan`: the jobs, their routing order, and per net the two grid states
and the cells opened for its ends) together with the static cells the
kernel was given. Cells are grid cells `(x, y)`.

A cell is *static* when the obstacle map blocks it (cells, rectangles,
keep-out band) or the hand-off reserved it (`StaticHandoffCells`: runways,
foreign-port keep-outs, fan-out stubs). A net may enter a static cell only
if it is one of the cells opened for that net.

Errors:

- `route_jobs.link_without_job`, `route_jobs.job_without_link`: the jobs
  are not exactly the links of the schematic (a job is its net name and its
  two `"instance,port"` ends)
- `route_jobs.net_ids_not_consecutive`: the net ids are not 1..n
- `route_jobs.order_not_a_permutation`: the routing order does not hold
  every job once
- `route_jobs.job_without_openings`: a job has no precomputed states
- `route_jobs.state_off_grid`: a source or target state lies outside the
  grid
- `route_jobs.state_blocked`: a state's cell is static and not opened for
  its net
- `route_jobs.state_heading_differs`: the source state does not head the
  way its port points, or the target state not the opposite way (headings
  in steps of 45 degrees)
- `route_jobs.ends_not_connected`: the target state cannot be reached from
  the source state through cells that are free or opened for the net, in
  steps to the eight neighbours; such a net cannot be routed whatever the
  search does. Other nets are not in the way here: the test is on the
  static cells alone.
"""

from __future__ import annotations

from collections import Counter
from typing import Any

import numpy as np
from scipy import ndimage

from photonic_router.logic_checks.findings import ERROR, Finding
from photonic_router.logic_checks.reading import schematic_links

Cell = tuple[int, int]

_NEIGHBOURS = tuple((dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0))


def check_route_jobs(schematic: Any, state: Any) -> list[Finding]:
    """The statements about the jobs and openings of a session after phase 5.

    `state` is the `SessionState` (`obstacles`, `static_cells`, `run`).
    """
    run = state.run
    jobs = run.route_jobs_by_id
    findings = _identity_findings(schematic, run)
    static = static_mask(state)
    areas = _label_free_areas(static)
    for net_id, job in sorted(jobs.items()):
        openings = run.state_openings_by_id.get(net_id)
        if openings is None:
            findings.append(Finding("route_jobs.job_without_openings", ERROR, job.net_name))
            continue
        findings.extend(_state_findings(job, openings, static, areas))
    return findings


def static_mask(state: Any) -> np.ndarray:
    """The static cells of a session as a mask indexed `[x, y]`."""
    obstacle_map = state.obstacles.obstacle_map
    grid = obstacle_map.grid
    mask = np.zeros((int(grid.width), int(grid.height)), dtype=bool)
    cells = (obstacle_map.blocked_cells, state.static_cells.blocked_cells_before_port_reservations)
    rects = (
        obstacle_map.blocked_static_rects,
        obstacle_map.chip_boundary_rects,
        state.static_cells.blocked_rects_for_diagnostics,
    )
    for group in cells:
        for x, y in group:
            if 0 <= x < mask.shape[0] and 0 <= y < mask.shape[1]:
                mask[x, y] = True
    for rect_group in rects:
        for x_min, y_min, x_max, y_max in rect_group:
            mask[max(x_min, 0) : x_max + 1, max(y_min, 0) : y_max + 1] = True
    return mask


def _identity_findings(schematic: Any, run: Any) -> list[Finding]:
    findings: list[Finding] = []
    jobs = run.route_jobs_by_id
    links = Counter(schematic_links(schematic))
    job_keys = Counter(
        (job.net_name, f"{job.inst1},{job.port1}", f"{job.inst2},{job.port2}")
        for job in jobs.values()
    )
    for net, first, second in (links - job_keys).elements():
        findings.append(Finding("route_jobs.link_without_job", ERROR, net, f"{first} -> {second}"))
    for net, first, second in (job_keys - links).elements():
        findings.append(Finding("route_jobs.job_without_link", ERROR, net, f"{first} -> {second}"))
    if sorted(jobs) != list(range(1, len(jobs) + 1)):
        findings.append(
            Finding("route_jobs.net_ids_not_consecutive", ERROR, "jobs", f"{len(jobs)} job(s)")
        )
    if sorted(run.route_order) != sorted(jobs):
        findings.append(
            Finding(
                "route_jobs.order_not_a_permutation",
                ERROR,
                "route order",
                f"{len(run.route_order)} entries for {len(jobs)} job(s)",
            )
        )
    return findings


def _state_findings(
    job: Any, openings: Any, static: np.ndarray, areas: np.ndarray
) -> list[Finding]:
    findings: list[Finding] = []
    opened: set[Cell] = openings.opened_cells_set
    cells: list[Cell] = []
    ends = (
        ("source", openings.source_state, job.source_port, 0),
        ("target", openings.target_state, job.target_port, 4),
    )
    for role, grid_state, port, turn in ends:
        cell = (int(grid_state.x), int(grid_state.y))
        subject = f"{job.net_name} {role}"
        if not (0 <= cell[0] < static.shape[0] and 0 <= cell[1] < static.shape[1]):
            findings.append(Finding("route_jobs.state_off_grid", ERROR, subject, f"{cell}"))
            continue
        cells.append(cell)
        if static[cell] and cell not in opened:
            findings.append(Finding("route_jobs.state_blocked", ERROR, subject, f"{cell}"))
        expected = (round(float(port.orientation) % 360.0 / 45.0) + turn) % 8
        if int(grid_state.angle) % 8 != expected:
            findings.append(
                Finding(
                    "route_jobs.state_heading_differs",
                    ERROR,
                    subject,
                    f"heading {int(grid_state.angle) % 8}, the port gives {expected}",
                )
            )
    if len(cells) == 2 and not _connected(cells[0], cells[1], opened, areas):
        findings.append(
            Finding(
                "route_jobs.ends_not_connected", ERROR, job.net_name, f"{cells[0]} -> {cells[1]}"
            )
        )
    return findings


def _label_free_areas(static: np.ndarray) -> np.ndarray:
    """Number the areas of free cells that hang together over the eight neighbours; 0 is static."""
    labels, _count = ndimage.label(~static, structure=np.ones((3, 3), dtype=bool))
    return np.asarray(labels)


def _connected(source: Cell, target: Cell, opened: set[Cell], areas: np.ndarray) -> bool:
    """Whether `target` is reachable from `source` over free areas and the net's opened cells.

    A free area is one node, every opened cell is one node; an opened cell
    touches its eight neighbours, opened or free.
    """
    start, goal = _node(source, opened, areas), _node(target, opened, areas)
    if start is None or goal is None:
        return False
    cells_at_area = _opened_cells_at_areas(opened, areas)
    seen: set[Cell | int] = {start}
    stack: list[Cell | int] = [start]
    while stack:
        current = stack.pop()
        if current == goal:
            return True
        if isinstance(current, int):
            adjacent: list[Cell | int | None] = list(cells_at_area.get(current, ()))
        else:
            adjacent = [
                _node(neighbour, opened, areas) for neighbour in _neighbours(current, areas.shape)
            ]
        for reached in adjacent:
            if reached is not None and reached not in seen:
                seen.add(reached)
                stack.append(reached)
    return False


def _node(cell: Cell, opened: set[Cell], areas: np.ndarray) -> Cell | int | None:
    """An opened cell is its own node, a free cell the number of its area; None is static."""
    if cell in opened:
        return cell
    return int(areas[cell]) or None


def _opened_cells_at_areas(opened: set[Cell], areas: np.ndarray) -> dict[int, list[Cell]]:
    """Per free area, the opened cells that touch it."""
    cells_at_area: dict[int, list[Cell]] = {}
    for cell in opened:
        for neighbour in _neighbours(cell, areas.shape):
            area = int(areas[neighbour])
            if area and neighbour not in opened:
                cells_at_area.setdefault(area, []).append(cell)
    return cells_at_area


def _neighbours(cell: Cell, shape: tuple[int, ...]) -> list[Cell]:
    return [
        (cell[0] + dx, cell[1] + dy)
        for dx, dy in _NEIGHBOURS
        if 0 <= cell[0] + dx < shape[0] and 0 <= cell[1] + dy < shape[1]
    ]
