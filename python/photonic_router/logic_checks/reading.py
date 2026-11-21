"""Stage "reading": the schematic of a benchmark and the layout placed from it.

`check_schematic` reads the netlist and the placements alone;
`check_layout` reads the placed instances and their ports
(`records.layout_from_schematic`). Lengths are in um, angles in degrees.

Errors (the design contradicts itself):

- `reading.instance_without_placement`, `reading.placement_without_instance`
- `reading.net_without_link`
- `reading.link_end_without_instance`: a link end is not `"instance,port"`
  with an instance of the netlist
- `reading.link_to_itself`: both ends of a link are the same port
- `reading.port_in_several_links`: a port can carry one waveguide
- `reading.layout_instances_differ`: the layout does not hold exactly the
  schematic's instances
- `reading.link_end_without_port`: the placed instance has no such port
- `reading.port_outside_instance`: a linked port lies outside the bounding
  box of its own instance
- `reading.port_not_axis_aligned`: a linked port points in a direction that
  is not a multiple of 90 degrees
- `reading.port_not_optical`: a linked port is not an optical port
- `reading.link_width_mismatch`, `reading.link_layer_mismatch`: the two
  ports of a link differ in width or layer
- `reading.link_ends_coincide`: the two ports of a link are at one point

Notes (legal; a later stage has to handle them):

- `reading.rotation_not_right_angle`: an instance placed at another angle
  than a multiple of 90 degrees
- `reading.instances_overlap`: the bounding boxes of two instances overlap
  with an area (an outline around the die does this on purpose)
- `reading.port_recessed`: a linked port lies inside its instance's
  bounding box, short of the face it points at, so the waveguide's first
  stretch runs over the instance's own box
- `reading.link_within_instance`: a link joins two ports of one instance
"""

from __future__ import annotations

import math
from collections import Counter
from typing import Any

from photonic_router.logic_checks.findings import ERROR, NOTE, Finding

TOLERANCE_UM = 1e-3
ANGLE_TOLERANCE_DEG = 1e-6

Link = tuple[str, str, str]
"""Net name and the two ends as written in the schematic."""


def schematic_links(schematic: Any) -> list[Link]:
    """Every link of `schematic.netlist.routes` as (net name, first end, second end)."""
    return [
        (str(net), str(first), str(second))
        for net, bundle in schematic.netlist.routes.items()
        for first, second in bundle.links.items()
    ]


def check_schematic(schematic: Any) -> list[Finding]:
    """The statements about the netlist and the placements alone."""
    findings: list[Finding] = []
    instances = schematic.netlist.instances
    placements = schematic.placements

    for name in instances:
        if name not in placements:
            findings.append(Finding("reading.instance_without_placement", ERROR, str(name)))
    for name, placement in placements.items():
        if name not in instances:
            findings.append(Finding("reading.placement_without_instance", ERROR, str(name)))
        rotation = float(placement.rotation or 0)
        if not _is_multiple_of_90(rotation):
            findings.append(
                Finding("reading.rotation_not_right_angle", NOTE, str(name), f"{rotation:g} deg")
            )

    for net, bundle in schematic.netlist.routes.items():
        if not bundle.links:
            findings.append(Finding("reading.net_without_link", ERROR, str(net)))

    findings.extend(_link_end_findings(schematic))
    return findings


def _link_end_findings(schematic: Any) -> list[Finding]:
    instances = schematic.netlist.instances
    findings: list[Finding] = []
    uses: Counter[str] = Counter()
    for net, first, second in schematic_links(schematic):
        for end in (first, second):
            uses[end] += 1
            if _split_end(end)[0] not in instances:
                findings.append(Finding("reading.link_end_without_instance", ERROR, net, end))
        if first == second:
            findings.append(Finding("reading.link_to_itself", ERROR, net, first))
    for end, count in uses.items():
        if count > 1:
            findings.append(Finding("reading.port_in_several_links", ERROR, end, f"{count} links"))
    return findings


def check_layout(schematic: Any, layout: Any) -> list[Finding]:
    """The statements about the placed instances and the ports the links name."""
    findings: list[Finding] = []
    placed = {str(instance.name): instance for instance in layout.insts}
    declared = {str(name) for name in schematic.netlist.instances}
    if set(placed) != declared:
        difference = ", ".join(sorted(set(placed) ^ declared))
        findings.append(Finding("reading.layout_instances_differ", ERROR, "layout", difference))

    boxes = {name: _box(instance) for name, instance in placed.items()}
    findings.extend(_overlaps(boxes))

    ports = {
        (name, str(port.name)): port for name, instance in placed.items() for port in instance.ports
    }
    for net, first, second in schematic_links(schematic):
        ends = []
        for end in (first, second):
            key = _split_end(end)
            if key[0] not in placed:
                continue  # reported by check_schematic or as layout_instances_differ
            if key not in ports:
                findings.append(Finding("reading.link_end_without_port", ERROR, net, end))
                continue
            ends.append((key, ports[key]))
            findings.extend(_port_findings(end, ports[key], boxes[key[0]]))
        if len(ends) == 2:
            findings.extend(_link_findings(net, ends[0], ends[1]))
    return findings


def _port_findings(end: str, port: Any, box: tuple[float, float, float, float]) -> list[Finding]:
    findings: list[Finding] = []
    left, bottom, right, top = box
    x, y = float(port.center[0]), float(port.center[1])
    if str(getattr(port, "port_type", "optical")) != "optical":
        findings.append(Finding("reading.port_not_optical", ERROR, end, str(port.port_type)))
    inside = (
        left - TOLERANCE_UM <= x <= right + TOLERANCE_UM
        and bottom - TOLERANCE_UM <= y <= top + TOLERANCE_UM
    )
    if not inside:
        findings.append(
            Finding("reading.port_outside_instance", ERROR, end, f"at ({x:.3f}, {y:.3f})")
        )
    orientation = float(port.orientation) % 360.0
    if not _is_multiple_of_90(orientation):
        findings.append(
            Finding("reading.port_not_axis_aligned", ERROR, end, f"{orientation:g} deg")
        )
        return findings
    to_face = {0: right - x, 90: top - y, 180: x - left, 270: y - bottom}[
        round(orientation / 90.0) % 4 * 90
    ]
    if inside and to_face > TOLERANCE_UM:
        findings.append(
            Finding("reading.port_recessed", NOTE, end, f"{to_face:.3f} um behind the face")
        )
    return findings


def _link_findings(
    net: str, first: tuple[tuple[str, str], Any], second: tuple[tuple[str, str], Any]
) -> list[Finding]:
    findings: list[Finding] = []
    (first_key, first_port), (second_key, second_port) = first, second
    if abs(float(first_port.width) - float(second_port.width)) > TOLERANCE_UM:
        findings.append(
            Finding(
                "reading.link_width_mismatch",
                ERROR,
                net,
                f"{float(first_port.width):g} / {float(second_port.width):g} um",
            )
        )
    first_layer, second_layer = _layer(first_port), _layer(second_port)
    if first_layer is not None and second_layer is not None and first_layer != second_layer:
        findings.append(
            Finding("reading.link_layer_mismatch", ERROR, net, f"{first_layer} / {second_layer}")
        )
    if math.dist(first_port.center, second_port.center) <= TOLERANCE_UM:
        findings.append(Finding("reading.link_ends_coincide", ERROR, net))
    if first_key[0] == second_key[0]:
        findings.append(Finding("reading.link_within_instance", NOTE, net, first_key[0]))
    return findings


def _overlaps(boxes: dict[str, tuple[float, float, float, float]]) -> list[Finding]:
    """Pairs of instances whose bounding boxes share an area, by a sweep over x."""
    findings: list[Finding] = []
    names = sorted(boxes, key=lambda name: (boxes[name][0], name))
    for index, name in enumerate(names):
        left, bottom, right, top = boxes[name]
        for other in names[index + 1 :]:
            other_left, other_bottom, other_right, other_top = boxes[other]
            if other_left >= right - TOLERANCE_UM:
                break
            width = min(right, other_right) - max(left, other_left)
            height = min(top, other_top) - max(bottom, other_bottom)
            if width > TOLERANCE_UM and height > TOLERANCE_UM:
                findings.append(
                    Finding(
                        "reading.instances_overlap",
                        NOTE,
                        f"{name} / {other}",
                        f"{width:.3f} x {height:.3f} um",
                    )
                )
    return findings


def _box(instance: Any) -> tuple[float, float, float, float]:
    box = instance.dbbox()
    return (float(box.left), float(box.bottom), float(box.right), float(box.top))


def _layer(port: Any) -> tuple[int, int] | None:
    info = getattr(port, "layer_info", None)
    return None if info is None else (int(info.layer), int(info.datatype))


def _split_end(end: str) -> tuple[str, str]:
    instance, _comma, port = end.partition(",")
    return instance, port


def _is_multiple_of_90(angle: float) -> bool:
    remainder = angle % 90.0
    return min(remainder, 90.0 - remainder) <= ANGLE_TOLERANCE_DEG
