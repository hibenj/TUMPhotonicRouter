"""Stage "routing": what the search hands back, before any correction of the ends.

`check_routing` reads the session state after the kernel batch (phase 6):
per net the committed route of the bookkeeping (`RoutedNetRecord` with the
kernel's `RouteResult`: its states, its segments and its cells), and the
crossings the kernel recorded. Cells are grid cells `(x, y)`, headings are
steps of 45 degrees.

A cell is *static* as in the stage "route jobs" (`route_jobs.static_mask`):
blocked by the obstacle map or reserved by the hand-off. A net may enter a
static cell only if it is one of the cells opened for that net.

Errors:

- `routing.job_without_route`, `routing.route_without_job`: the routed nets
  are not exactly the jobs
- `routing.route_misses_source`: the route does not start in the cell and
  with the heading of the job's source state
- `routing.route_misses_target`: the route does not end in the cell and
  with the heading of the job's target state
- `routing.segments_not_chained`: a segment does not start in the cell and
  with the heading the segment before it ended in
- `routing.segment_length_wrong`: a straight segment's length is not the
  distance of its two cells, or a bend is shorter than the distance of its
  two cells
- `routing.length_not_the_segments`: the route's length is not the sum of
  its segments' lengths, or the recorded length of the net is not the
  route's
- `routing.cells_not_a_path`: the route's cells do not run from the source
  cell to the target cell in steps to one of the eight neighbours
- `routing.route_through_static`: a cell of the route is outside the grid
  or static and not opened for the net
- `routing.nets_meet_without_crossing`: the cell paths of two nets meet (in
  a cell both hold, or in a unit square they pass along its two diagonals)
  and no crossing of the two is recorded
- `routing.crossing_without_meeting`: a crossing is recorded for two nets
  whose cell paths never meet
- `routing.crossing_not_legal`: a crossing the kernel recorded has a
  verdict that is not legal
- `routing.crossing_without_routes`: a recorded crossing names a net that
  has no route

When the kernel gives up on a net the flow raises before this stage is
reached; `scripts/logic_check.py` reports that run as
`routing.net_not_routed`.
"""

from __future__ import annotations

import math
from collections import Counter
from itertools import combinations, pairwise
from typing import Any

import numpy as np

from photonic_router.logic_checks.findings import ERROR, Finding
from photonic_router.logic_checks.route_jobs import static_mask

Cell = tuple[int, int]

LENGTH_TOLERANCE_UM = 1e-6


def check_routing(state: Any) -> list[Finding]:
    """The statements about the routes of a session after the kernel batch.

    `state` is the `SessionState` (`run`, `search.bookkeeping`,
    `kernel.router`, `obstacles`, `static_cells`).
    """
    jobs = state.run.route_jobs_by_id
    records = state.search.bookkeeping.records_by_id
    findings = [
        Finding("routing.job_without_route", ERROR, jobs[net_id].net_name)
        for net_id in sorted(set(jobs) - set(records))
    ]
    findings.extend(
        Finding("routing.route_without_job", ERROR, str(records[net_id].net_name))
        for net_id in sorted(set(records) - set(jobs))
    )
    static = static_mask(state)
    grid_size_um = float(state.obstacles.obstacle_map.grid.grid_size_um)
    cells_by_net: dict[int, list[Cell]] = {}
    for net_id in sorted(set(jobs) & set(records)):
        record = records[net_id]
        route = record.route_obj
        openings = state.run.state_openings_by_id[net_id]
        cells = [(int(x), int(y)) for x, y in route.cells]
        cells_by_net[net_id] = cells
        findings.extend(_end_findings(record.net_name, route, openings))
        findings.extend(_segment_findings(record, route, grid_size_um))
        findings.extend(_cell_findings(record.net_name, cells, openings, static))
    events = list(state.kernel.router.crossing_events())
    findings.extend(_meeting_findings(cells_by_net, events, records))
    findings.extend(_crossing_findings(state.kernel.router, records))
    return findings


def _end_findings(net_name: str, route: Any, openings: Any) -> list[Finding]:
    states = list(route.states)
    if not states:
        return [Finding("routing.route_misses_source", ERROR, net_name, "the route has no state")]
    findings: list[Finding] = []
    for check, found, wanted in (
        ("routing.route_misses_source", states[0], openings.source_state),
        ("routing.route_misses_target", states[-1], openings.target_state),
    ):
        if _pose(found) != _pose(wanted):
            findings.append(
                Finding(check, ERROR, net_name, f"{_pose(found)}, the job has {_pose(wanted)}")
            )
    return findings


def _pose(state: Any) -> tuple[int, int, int]:
    return (int(state.x), int(state.y), int(state.angle) % 8)


def _segment_findings(record: Any, route: Any, grid_size_um: float) -> list[Finding]:
    net_name = record.net_name
    segments = [dict(segment) for segment in route.segments]
    findings: list[Finding] = []
    for index, (before, after) in enumerate(pairwise(segments), start=1):
        if (
            tuple(before["end"]) != tuple(after["start"])
            or int(before["end_angle"]) % 8 != int(after["start_angle"]) % 8
        ):
            findings.append(
                Finding(
                    "routing.segments_not_chained",
                    ERROR,
                    net_name,
                    f"segment {index}: ends at {before['end']} heading {before['end_angle']}, "
                    f"the next starts at {after['start']} heading {after['start_angle']}",
                )
            )
    for index, segment in enumerate(segments):
        chord_um = math.dist(segment["start"], segment["end"]) * grid_size_um
        length_um = float(segment["length_um"])
        straight = str(segment["kind"]) == "straight"
        if (straight and abs(length_um - chord_um) > LENGTH_TOLERANCE_UM) or (
            length_um < chord_um - LENGTH_TOLERANCE_UM
        ):
            findings.append(
                Finding(
                    "routing.segment_length_wrong",
                    ERROR,
                    net_name,
                    f"segment {index} ({segment['kind']}): {length_um:.6f} um for a distance "
                    f"of {chord_um:.6f} um",
                )
            )
    total_um = sum(float(segment["length_um"]) for segment in segments)
    route_um = float(route.total_length_um)
    if abs(route_um - total_um) > LENGTH_TOLERANCE_UM * max(1, len(segments)):
        findings.append(
            Finding(
                "routing.length_not_the_segments",
                ERROR,
                net_name,
                f"route {route_um:.6f} um, segments {total_um:.6f} um",
            )
        )
    recorded_um = (
        record.base_total_length_um
        if record.base_total_length_um is not None
        else record.total_length_um
    )
    if abs(float(recorded_um) - route_um) > LENGTH_TOLERANCE_UM:
        findings.append(
            Finding(
                "routing.length_not_the_segments",
                ERROR,
                net_name,
                f"recorded {float(recorded_um):.6f} um, route {route_um:.6f} um",
            )
        )
    return findings


def _cell_findings(
    net_name: str, cells: list[Cell], openings: Any, static: np.ndarray
) -> list[Finding]:
    findings: list[Finding] = []
    source = _pose(openings.source_state)[:2]
    target = _pose(openings.target_state)[:2]
    steps_ok = all(max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1 for a, b in pairwise(cells))
    if not cells or cells[0] != source or cells[-1] != target or not steps_ok:
        findings.append(
            Finding(
                "routing.cells_not_a_path",
                ERROR,
                net_name,
                f"{len(cells)} cell(s) for {source} -> {target}",
            )
        )
    opened = openings.opened_cells_set
    width, height = static.shape
    bad = [
        cell
        for cell in cells
        if not (0 <= cell[0] < width and 0 <= cell[1] < height)
        or (static[cell] and cell not in opened)
    ]
    if bad:
        findings.append(
            Finding(
                "routing.route_through_static",
                ERROR,
                net_name,
                f"{len(bad)} cell(s), first {bad[0]}",
            )
        )
    return findings


def _meetings(cells_by_net: dict[int, list[Cell]]) -> Counter[frozenset[int]]:
    """Per pair of nets, the places where their cell paths meet.

    Two paths meet in a cell both hold, and in a unit square that one passes
    along one diagonal and the other along the other diagonal.
    """
    owners: dict[tuple[int, int, int], set[int]] = {}
    for net_id, cells in cells_by_net.items():
        for cell in cells:
            owners.setdefault((cell[0], cell[1], 0), set()).add(net_id)
        for (x0, y0), (x1, y1) in pairwise(cells):
            if x0 != x1 and y0 != y1:
                rising = (x1 - x0) == (y1 - y0)
                owners.setdefault((min(x0, x1), min(y0, y1), 1 if rising else 2), set()).add(net_id)
    meetings: Counter[frozenset[int]] = Counter()
    for (x, y, kind), nets in owners.items():
        if kind == 0:
            meetings.update(frozenset(pair) for pair in combinations(sorted(nets), 2))
        elif kind == 1:
            meetings.update(
                frozenset((a, b)) for a in nets for b in owners.get((x, y, 2), ()) if a != b
            )
    return meetings


def _meeting_findings(
    cells_by_net: dict[int, list[Cell]], events: list[Any], records: dict[int, Any]
) -> list[Finding]:
    crossing_pairs = {
        frozenset((int(event["net_id"]), int(event["partner_net_id"]))) for event in events
    }
    meetings = _meetings(cells_by_net)

    def name(pair: frozenset[int]) -> str:
        return " x ".join(
            sorted(
                str(records[net_id].net_name) if net_id in records else f"net id {net_id}"
                for net_id in pair
            )
        )

    findings = [
        Finding(
            "routing.nets_meet_without_crossing",
            ERROR,
            name(pair),
            f"{count} place(s), no crossing recorded",
        )
        for pair, count in sorted(meetings.items(), key=lambda item: sorted(item[0]))
        if pair not in crossing_pairs
    ]
    findings.extend(
        Finding("routing.crossing_without_meeting", ERROR, name(pair))
        for pair in sorted(crossing_pairs - set(meetings), key=sorted)
        if all(net_id in cells_by_net for net_id in pair)
    )
    return findings


def _crossing_findings(router: Any, records: dict[int, Any]) -> list[Finding]:
    findings: list[Finding] = []
    for verdict in router.crossing_verdicts():
        pair = (int(verdict["net_id"]), int(verdict["partner_net_id"]))
        names = " x ".join(
            str(records[net_id].net_name) if net_id in records else f"net id {net_id}"
            for net_id in pair
        )
        if any(net_id not in records for net_id in pair):
            findings.append(Finding("routing.crossing_without_routes", ERROR, names))
        if not verdict["legal"]:
            findings.append(
                Finding("routing.crossing_not_legal", ERROR, names, str(verdict["reason"]))
            )
    return findings
