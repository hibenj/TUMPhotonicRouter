"""Stage "realized": the polygons of the routed layout against the records they were made from.

`check_realized` reads the `RealizedNets` that `route_match_and_realize`
reports (`routing.stage_observer`): the routed layout, the records, the
route layer and width, and the crossing plan info with the crossing
components that were placed. A net's *polygons* are the polygons of the
layout's top cell on the route layer that hold more of this net's
centerline than of any other; lengths and coordinates are in um.

A net without crossing is one polygon from port to port. At a crossing
component both nets are cut: the component carries the light across, so a
net with k crossings is k + 1 polygons and its centerline runs through k
component footprints. A net with a meander (path-length matching) carries
the centerline from before the meander and the meander as a plan
(`meander_auto_plan`); its polygon is longer than its recorded length by
the extra length the plan asked for.

`check_verified` reads phase 8's product before that: the records after
the repair loops and the realized crossings that are still illegal.

Errors:

- `realized.illegal_crossing_left`: phase 8 hands on a realized crossing
  that is still illegal after its repair rounds
- `realized.polygon_not_simple`: a polygon on the route layer is not a
  valid simple polygon
- `realized.net_without_polygon`: a record has no polygon
- `realized.polygon_without_net`: a polygon holds no record's centerline
- `realized.pieces_not_the_crossings`: a net does not have one polygon more
  than it has crossing components
- `realized.port_not_in_polygon`: the centre of a net's source or target
  port lies in none of the net's polygons
- `realized.centerline_uncovered`: a stretch of a centerline (of a net
  without meander) lies neither in the net's polygons nor in the footprint
  of one of its crossing components
- `realized.length_not_the_record`: the length of a net's polygons, their
  area divided by the route width, is not the length of the centerline
  inside them or, for a net with a meander, not the recorded length plus
  the extra length of the plan (to `LENGTH_TOLERANCE`, relative)
- `realized.nets_overlap`: polygons of two nets overlap

Note:

- `realized.net_with_meander`: a net whose polygon carries a meander
"""

from __future__ import annotations

from collections import Counter
from typing import Any

import shapely

from photonic_router.logic_checks.findings import ERROR, NOTE, Finding
from photonic_router.logic_checks.matching import check_layout_matching

EDGE_TOLERANCE_UM = 2e-3
"""Polygons are on the layout's database grid (1 nm); a centerline is not."""
UNCOVERED_TOLERANCE_UM = 1e-2
LENGTH_TOLERANCE = 5e-3
"""Relative; an arc is drawn as a polygon, 0.11 % is the most seen."""
OVERLAP_TOLERANCE_UM2 = 1e-6


def route_layer_polygons(layout: Any, route_layer: tuple[int, int]) -> list[shapely.Polygon]:
    """The polygons of the layout's top cell on the route layer, in um."""
    dbu = float(layout.kcl.dbu)
    layer_index = layout.kcl.layout.layer(*route_layer)
    polygons = []
    for shape in layout.kdb_cell.shapes(layer_index).each():
        polygon = shape.polygon
        if polygon is None:
            continue
        hull = [(point.x * dbu, point.y * dbu) for point in polygon.each_point_hull()]
        holes = [
            [(point.x * dbu, point.y * dbu) for point in polygon.each_point_hole(index)]
            for index in range(polygon.holes())
        ]
        polygons.append(shapely.Polygon(hull, holes))
    return polygons


def check_verified(verified: Any) -> list[Finding]:
    """The statement about phase 8's product (a `VerifiedRecords`)."""
    return [
        Finding(
            "realized.illegal_crossing_left",
            ERROR,
            f"{item.get('net_name_a')} x {item.get('net_name_b')}",
            str(item.get("reason")),
        )
        for item in verified.illegal_realized_crossings
    ]


def check_realized(realized: Any) -> list[Finding]:
    """The statements about the realized polygons of `realized` (a `RealizedNets`)."""
    polygons = route_layer_polygons(realized.routed_layout, realized.route_layer)
    info = realized.crossing_plan_info
    components = (info.realized_crossing_components if info is not None else None) or []
    width_um = float(realized.route_width_um)
    findings = check_polygons(polygons, realized.records, width_um, components)
    if realized.matched is not None:
        lengths = polygon_lengths_um(polygons, realized.records, width_um, components)
        findings.extend(check_layout_matching(realized.matched, lengths))
    return findings


def polygon_lengths_um(
    polygons: list[shapely.Polygon],
    records: list[Any],
    route_width_um: float,
    crossing_components: list[Any],
) -> dict[str, float]:
    """Per net name, the length drawn on the route layer: polygon area by route width.

    Only the nets whose polygons hold their whole length: a net through a
    crossing component is left out (the component's footprint is not a
    route polygon), and so is a net without a polygon.
    """
    valid = [polygon for polygon in polygons if polygon.is_valid]
    lines = [shapely.LineString(record.corrected_centerline_um) for record in records]
    pieces = _pieces_by_record(valid, lines)
    crossed = set(_footprints_by_net(crossing_components))
    return {
        str(record.net_name): sum(float(valid[index].area) for index in pieces[number])
        / route_width_um
        for number, record in enumerate(records)
        if number in pieces and str(record.net_name) not in crossed
    }


def check_polygons(
    polygons: list[shapely.Polygon],
    records: list[Any],
    route_width_um: float,
    crossing_components: list[Any],
) -> list[Finding]:
    """`check_realized` on polygons that were already read from the layout.

    `crossing_components` are the records of the crossing components that
    were placed (`CrossingPlanInfo.realized_crossing_components`).
    """
    findings = [
        Finding("realized.polygon_not_simple", ERROR, f"polygon {index}", _where(polygon))
        for index, polygon in enumerate(polygons)
        if not polygon.is_valid
    ]
    polygons = [polygon for polygon in polygons if polygon.is_valid]
    lines = [shapely.LineString(record.corrected_centerline_um) for record in records]
    pieces = _pieces_by_record(polygons, lines)
    owned = {index for indices in pieces.values() for index in indices}
    findings.extend(
        Finding("realized.polygon_without_net", ERROR, f"polygon {index}", _where(polygon))
        for index, polygon in enumerate(polygons)
        if index not in owned
    )
    footprints = _footprints_by_net(crossing_components)
    for number, (record, line) in enumerate(zip(records, lines, strict=True)):
        own = [polygons[index] for index in pieces.get(number, ())]
        if not own:
            findings.append(Finding("realized.net_without_polygon", ERROR, record.net_name))
            continue
        findings.extend(
            _net_findings(
                record, line, own, footprints.get(str(record.net_name), []), route_width_um
            )
        )
    findings.extend(_overlap_findings(polygons, pieces, records))
    return findings


def _where(polygon: shapely.Polygon) -> str:
    x_min, y_min, x_max, y_max = polygon.bounds
    return f"box ({x_min:.3f}, {y_min:.3f}) .. ({x_max:.3f}, {y_max:.3f})"


def _pieces_by_record(
    polygons: list[shapely.Polygon], lines: list[shapely.LineString]
) -> dict[int, list[int]]:
    """Per record number, the polygons that hold more of its centerline than of any other."""
    tree = shapely.STRtree(lines)
    pieces: dict[int, list[int]] = {}
    for index, polygon in enumerate(polygons):
        grown = polygon.buffer(EDGE_TOLERANCE_UM)
        best, best_length = None, 0.0
        for number in sorted(int(candidate) for candidate in tree.query(grown)):
            length = float(grown.intersection(lines[number]).length)
            if length > best_length:
                best, best_length = number, length
        if best is not None:
            pieces.setdefault(best, []).append(index)
    return pieces


def _footprints_by_net(crossing_components: list[Any]) -> dict[str, list[shapely.Polygon]]:
    """Per net name, the footprints of its crossing components."""
    footprints: dict[str, list[shapely.Polygon]] = {}
    for component in crossing_components:
        footprint = shapely.Polygon(component["crossing_footprint_polygon_um"])
        for key in ("net_name_a", "net_name_b"):
            footprints.setdefault(str(component[key]), []).append(footprint)
    return footprints


def _net_findings(
    record: Any,
    line: shapely.LineString,
    own: list[shapely.Polygon],
    footprints: list[shapely.Polygon],
    route_width_um: float,
) -> list[Finding]:
    name = record.net_name
    findings: list[Finding] = []
    if len(own) != len(footprints) + 1:
        findings.append(
            Finding(
                "realized.pieces_not_the_crossings",
                ERROR,
                name,
                f"{len(own)} polygon(s), {len(footprints)} crossing component(s)",
            )
        )
    body = shapely.union_all([polygon.buffer(EDGE_TOLERANCE_UM) for polygon in own])
    for role, centre in (
        ("source", record.source_port_center_um),
        ("target", record.target_port_center_um),
    ):
        if centre is not None and not body.contains(shapely.Point(centre)):
            findings.append(Finding("realized.port_not_in_polygon", ERROR, f"{name} {role}"))
    polygon_um = sum(float(polygon.area) for polygon in own) / route_width_um
    plan = record.meander_auto_plan
    if plan is not None:
        findings.append(Finding("realized.net_with_meander", NOTE, name))
        expected_um = float(record.total_length_um) + float(plan["requested_extra_length_um"])
        expected = "the recorded length and the meander's extra length"
    else:
        covered = shapely.union_all(
            [body, *(footprint.buffer(EDGE_TOLERANCE_UM) for footprint in footprints)]
        )
        uncovered_um = float(line.difference(covered).length)
        if uncovered_um > UNCOVERED_TOLERANCE_UM:
            findings.append(
                Finding(
                    "realized.centerline_uncovered",
                    ERROR,
                    name,
                    f"{uncovered_um:.3f} um outside its polygons and crossing footprints",
                )
            )
        expected_um = float(line.intersection(body).length)
        expected = "the centerline inside them"
    if abs(polygon_um - expected_um) > LENGTH_TOLERANCE * max(expected_um, 1.0):
        findings.append(
            Finding(
                "realized.length_not_the_record",
                ERROR,
                name,
                f"polygons {polygon_um:.3f} um, {expected} {expected_um:.3f} um",
            )
        )
    return findings


def _overlap_findings(
    polygons: list[shapely.Polygon], pieces: dict[int, list[int]], records: list[Any]
) -> list[Finding]:
    owner = {index: number for number, indices in pieces.items() for index in indices}
    tree = shapely.STRtree(polygons)
    overlaps: Counter[tuple[int, int]] = Counter()
    for index, polygon in enumerate(polygons):
        for other in sorted(int(candidate) for candidate in tree.query(polygon)):
            if other <= index or owner.get(other) == owner.get(index):
                continue
            if index not in owner or other not in owner:
                continue
            if float(polygon.intersection(polygons[other]).area) > OVERLAP_TOLERANCE_UM2:
                first, second = sorted((owner[index], owner[other]))
                overlaps[(first, second)] += 1
    return [
        Finding(
            "realized.nets_overlap",
            ERROR,
            f"{records[first].net_name} x {records[second].net_name}",
            f"{count} pair(s) of polygons",
        )
        for (first, second), count in sorted(overlaps.items())
    ]
