"""Stage "records": the routed nets after the ends were corrected to the ports.

`check_records` reads the `RoutedNetRecord` list phase 7 hands on
(`finalize`, after the optional detour pass). The kernel routes between
grid cells; the endpoint correction then bends each end of a route onto
the centre of its port and stores the result as the record's
`corrected_centerline_um`, a polyline in um from the source port to the
target port.

Errors:

- `records.link_without_record`, `records.record_without_link`: the
  records are not exactly the links of the schematic (a record is its net
  name and its two `"instance,port"` ends)
- `records.correction_failed`: a record carries the message of a failed
  endpoint correction
- `records.centerline_misses_port`: a corrected centerline does not start
  on the centre of the source port or does not end on the centre of the
  target port (to 1 nm)
- `records.centerline_point_repeated`: two successive points of a
  centerline are the same
- `records.centerline_leaves_port_askew`: the first step of a centerline
  does not run the way the source port points, or the last step does not
  run against the way the target port points (to `ASKEW_TOLERANCE_DEG`)
- `records.length_not_the_centerline`: the recorded length of a corrected
  net is not the length of its centerline
- `records.base_length_not_the_route`: a corrected net does not keep the
  length of its kernel route as its base length

Note:

- `records.end_not_corrected`: a record without a corrected centerline; its
  ends are where the grid put them
"""

from __future__ import annotations

import math
from collections import Counter
from itertools import pairwise
from typing import Any

from photonic_router.logic_checks.findings import ERROR, NOTE, Finding
from photonic_router.logic_checks.reading import schematic_links

PORT_TOLERANCE_UM = 1e-3
LENGTH_TOLERANCE_UM = 1e-6
ASKEW_TOLERANCE_DEG = 5.0

Point = tuple[float, float]


def check_records(schematic: Any, records: list[Any]) -> list[Finding]:
    """The statements about the routed net records of a session after phase 7."""
    links = Counter(schematic_links(schematic))
    recorded = Counter(
        (
            str(record.net_name),
            f"{record.source.instance},{record.source.port}",
            f"{record.target.instance},{record.target.port}",
        )
        for record in records
    )
    findings = [
        Finding("records.link_without_record", ERROR, net, f"{first} -> {second}")
        for net, first, second in (links - recorded).elements()
    ]
    findings.extend(
        Finding("records.record_without_link", ERROR, net, f"{first} -> {second}")
        for net, first, second in (recorded - links).elements()
    )
    for record in records:
        if record.endpoint_correction_error is not None:
            findings.append(
                Finding(
                    "records.correction_failed",
                    ERROR,
                    record.net_name,
                    str(record.endpoint_correction_error),
                )
            )
        if not record.corrected_centerline_um:
            findings.append(Finding("records.end_not_corrected", NOTE, record.net_name))
            continue
        findings.extend(_centerline_findings(record))
        findings.extend(_length_findings(record))
    return findings


def _centerline_findings(record: Any) -> list[Finding]:
    name = record.net_name
    line: list[Point] = [(float(x), float(y)) for x, y in record.corrected_centerline_um]
    findings: list[Finding] = []
    ends = (
        ("source", line[0], record.source_port_center_um),
        ("target", line[-1], record.target_port_center_um),
    )
    for role, point, port in ends:
        if port is not None and math.dist(point, port) > PORT_TOLERANCE_UM:
            findings.append(
                Finding(
                    "records.centerline_misses_port",
                    ERROR,
                    f"{name} {role}",
                    f"{math.dist(point, port):.4f} um from the port centre",
                )
            )
    repeated = sum(1 for a, b in pairwise(line) if a == b)
    if repeated or len(line) < 2:
        findings.append(
            Finding(
                "records.centerline_point_repeated",
                ERROR,
                name,
                f"{repeated} repeated point(s) among {len(line)}",
            )
        )
        return findings
    # leaving the source the way it points; arriving against the way the target points
    steps = (
        ("source", line[0], line[1], record.source_port_orientation_deg),
        ("target", line[-1], line[-2], record.target_port_orientation_deg),
    )
    for role, port_point, next_point, orientation in steps:
        if orientation is None:
            continue
        heading = math.degrees(
            math.atan2(next_point[1] - port_point[1], next_point[0] - port_point[0])
        )
        off = abs(heading - float(orientation)) % 360.0
        off = min(off, 360.0 - off)
        if off > ASKEW_TOLERANCE_DEG:
            findings.append(
                Finding(
                    "records.centerline_leaves_port_askew",
                    ERROR,
                    f"{name} {role}",
                    f"{off:.2f} degrees off the port's direction",
                )
            )
    return findings


def _length_findings(record: Any) -> list[Finding]:
    name = record.net_name
    findings: list[Finding] = []
    line_um = sum(math.dist(a, b) for a, b in pairwise(record.corrected_centerline_um))
    if abs(float(record.total_length_um) - line_um) > LENGTH_TOLERANCE_UM:
        findings.append(
            Finding(
                "records.length_not_the_centerline",
                ERROR,
                name,
                f"recorded {float(record.total_length_um):.6f} um, centerline {line_um:.6f} um",
            )
        )
    route_um = float(record.route_obj.total_length_um)
    base_um = record.base_total_length_um
    if base_um is None or abs(float(base_um) - route_um) > LENGTH_TOLERANCE_UM:
        findings.append(
            Finding(
                "records.base_length_not_the_route",
                ERROR,
                name,
                f"base {base_um}, route {route_um:.6f} um",
            )
        )
    return findings
