"""Stage "electrical": the heater wiring the electrical router hands on.

`check_electrical` reads the `ElectricalRoutingResult` of the flow's
electrical step. Every heater has two terminals: one is joined to the
common bus, the other gets a wire of its own to a pad. The wires are
paths on the electrical grid.

Errors:

- `electrical.heater_not_on_bus`: the common bus router gave up on a heater
- `electrical.heater_without_bus_terminal`: a heater is neither on the bus
  nor reported as failed
- `electrical.bus_terminal_not_of_heater`: the terminal chosen for the bus,
  or the one left for the wire, is not one of the heater's two, or both
  are the same terminal
- `electrical.wire_not_routed`: a pad wire the detail router reports as
  failed
- `electrical.heater_without_wire`, `electrical.heater_with_several_wires`:
  the terminal a heater keeps for its own wire has no wire, or more than
  one
- `electrical.pad_shared`: two nets are assigned the same pad slot
- `electrical.wire_without_pad`: a routed wire has no pad assignment
- `electrical.wire_not_a_path`: two successive cells of a wire are not
  neighbours, or a wire has no cells
- `electrical.wire_misses_pad`: a wire does not end on a cell of its pad
- `electrical.wires_share_cell`: a cell lies on the paths of two wires
- `electrical.own_check_failed`: the electrical router's own verification
  reports an error (one finding per error, named by its code)

The stage has no findings when the run made no pad plan (the result then
holds the common bus only).
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping
from itertools import pairwise
from typing import Any

from photonic_router.logic_checks.findings import ERROR, Finding


def check_electrical(result: Any) -> list[Finding]:
    """The statements about the wiring of the heaters."""
    return [
        *_check_common_bus(result),
        *_check_wires(result),
        *_check_own_verification(result),
    ]


def _check_common_bus(result: Any) -> list[Finding]:
    bus = result.common_bus
    failed = set(bus.failed_heaters)
    findings = [Finding("electrical.heater_not_on_bus", ERROR, str(h)) for h in bus.failed_heaters]
    for group in result.terminal_groups:
        if group.heater_id not in failed:
            findings += _check_terminals_of(group, bus)
    return findings


def _check_terminals_of(group: Any, bus: Any) -> list[Finding]:
    """One heater: the terminal on the bus and the one for the wire are its two."""
    heater = group.heater_id
    on_bus = bus.selected_terminals.get(heater)
    for_wire = bus.unselected_terminals.get(heater)
    if on_bus is None:
        return [Finding("electrical.heater_without_bus_terminal", ERROR, heater)]
    own = {group.terminal_a.id, group.terminal_b.id}
    if on_bus.id in own and for_wire is not None and for_wire.id in own - {on_bus.id}:
        return []
    detail = f"bus {on_bus.id}, wire {getattr(for_wire, 'id', None)}, heater has {sorted(own)}"
    return [Finding("electrical.bus_terminal_not_of_heater", ERROR, heater, detail)]


def _check_wires(result: Any) -> list[Finding]:
    detail = result.detailed_bundle_routes
    if detail is None:
        return []
    return [
        *(
            Finding("electrical.wire_not_routed", ERROR, route.terminal.id, str(route.reason or ""))
            for route in detail.failed_routes
        ),
        *_check_one_wire_per_heater(result.common_bus.unselected_terminals, detail),
        *_check_pads(result.pad_plan),
        *_check_paths(detail.routes),
    ]


def _check_one_wire_per_heater(for_wire: Mapping[str, Any], detail: Any) -> list[Finding]:
    failed = {route.terminal.id for route in detail.failed_routes}
    wires = Counter(route.terminal.id for route in detail.routes)
    findings = []
    for heater, terminal in for_wire.items():
        count = wires[terminal.id]
        if terminal.id in failed or count == 1:
            continue
        check = "heater_without_wire" if count == 0 else "heater_with_several_wires"
        findings.append(Finding(f"electrical.{check}", ERROR, heater, terminal.id))
    return findings


def _check_pads(pad_plan: Any) -> list[Finding]:
    if pad_plan is None:
        return []
    slots = Counter(assignment.slot.index for assignment in pad_plan.assignments)
    return [
        Finding("electrical.pad_shared", ERROR, f"slot {index}", f"{count} nets")
        for index, count in sorted(slots.items())
        if count > 1
    ]


def _check_paths(routes: Any) -> list[Finding]:
    findings = []
    on_cell: dict[tuple[int, int], str] = {}
    for route in routes:
        name = route.terminal.id
        path = [(int(cell[0]), int(cell[1])) for cell in route.path]
        findings += _check_path(route, path)
        for cell in set(path):
            other = on_cell.setdefault(cell, name)
            if other != name:
                findings.append(
                    Finding("electrical.wires_share_cell", ERROR, name, f"{cell} with {other}")
                )
    return findings


def _check_path(route: Any, path: list[tuple[int, int]]) -> list[Finding]:
    """One wire: it has a pad, its cells are a path, and the path ends on the pad."""
    name = route.terminal.id
    findings = []
    if route.pad_assignment is None:
        findings.append(Finding("electrical.wire_without_pad", ERROR, name))
    if not path or any(max(abs(a[0] - b[0]), abs(a[1] - b[1])) != 1 for a, b in pairwise(path)):
        return [*findings, Finding("electrical.wire_not_a_path", ERROR, name)]
    if path[-1] not in route.target_cells and path[0] not in route.target_cells:
        findings.append(Finding("electrical.wire_misses_pad", ERROR, name, f"ends {path[-1]}"))
    return findings


def _check_own_verification(result: Any) -> list[Finding]:
    verification = result.verification
    if verification is None:
        return []
    return [
        Finding("electrical.own_check_failed", ERROR, str(issue.code), str(issue.message)[:200])
        for issue in verification.issues
        if issue.severity == "error"
    ]
