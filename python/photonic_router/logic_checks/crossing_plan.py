"""Stage "crossing plan": the crossings a placement forces, and what the session takes of them.

`check_crossing_demand` reads the `CrossingDemand` of the placement
analysis (`crossings/crossing_demand.py`) against the schematic and the
placed layout it was built from. A *band* is the space between two
neighbouring component columns; a net is *confined* to a band when its two
ends point into it from its two walls. `check_crossing_plan_info` reads
what phase 4 of a session made of the demand (`CrossingPlanInfo`).

Errors of the demand:

- `crossing_plan.link_not_accounted`: a link of the schematic is neither
  confined to a band nor listed as outside the band model, or it is both,
  or it is confined to two bands
- `crossing_plan.net_not_a_link`: the demand names a net that is no link of
  the schematic
- `crossing_plan.band_without_width`: a band's left face is not left of its
  right face
- `crossing_plan.wall_orders_differ`: the two wall orders of a band do not
  hold the same nets, each once
- `crossing_plan.end_not_on_wall`: a confined net has no end that points
  right from the band's left wall and one that points left from its right
  wall (an end is on a wall when it lies at the wall's face or behind it)
- `crossing_plan.wall_order_not_top_to_bottom`: along a wall order the
  heights of the ends rise somewhere
- `crossing_plan.pairs_not_the_inversions`: the invariant pairs of a band
  are not exactly the pairs of nets whose order on the left wall is the
  reverse of their order on the right wall
- `crossing_plan.events_not_the_pairs`: the crossing events of a band are
  not its invariant pairs, each once
- `crossing_plan.events_do_not_sort`: taken in turn as swaps of two
  neighbouring nets, upper net first, the events do not turn the left wall
  order into the right wall order

Notes of the demand:

- `crossing_plan.net_outside_band_model`: a link the band model does not
  cover (no crossing is planned for it); the detail is the reason
- `crossing_plan.outside_net_without_path`: a link between non-adjacent
  walls for which the region graph has no way through the passages

Errors of the session's plan:

- `crossing_plan.baseline_reads_plan`: the mode is `lidar-pure` and the
  plan info holds events, expected counts, a plan text or guidance; the
  baseline must not be informed by a plan
- `crossing_plan.event_without_job`: a planned event names a net that has
  no route job
- `crossing_plan.guidance_not_the_plan`: in `lidar-guided` mode without a
  net filter, the pairs handed to the router are not the invariant pairs of
  the demand, as net ids of the jobs
- `crossing_plan.expected_counts_differ`: the expected crossings per net
  are not the number of guided pairs the net is in
"""

from __future__ import annotations

from collections import Counter
from itertools import combinations, pairwise
from typing import Any

from photonic_router.logic_checks.findings import ERROR, NOTE, Finding
from photonic_router.logic_checks.reading import schematic_links

TOLERANCE_UM = 1e-3
ANGLE_TOLERANCE_DEG = 1.0
NON_ADJACENT_WALLS = "non_adjacent_walls"
GUIDED_MODE = "lidar-guided"

Link = tuple[str, str, str]
"""A net name and its two `"instance,port"` ends, as `schematic_links` gives them."""


def check_crossing_demand(schematic: Any, layout: Any, demand: Any) -> list[Finding]:
    """The statements about a `CrossingDemand` of `schematic` placed as `layout`."""
    findings = _accounting_findings(schematic, demand)
    ends = _placed_ends(layout)
    for band in demand.bands.values():
        findings.extend(_band_findings(band, ends))
    findings.extend(
        Finding("crossing_plan.net_outside_band_model", NOTE, net.edge_key.net_name, net.reason)
        for net in demand.outside
    )
    findings.extend(
        Finding("crossing_plan.outside_net_without_path", NOTE, net.edge_key.net_name)
        for net in demand.outside
        if net.reason == NON_ADJACENT_WALLS and net.edge_key not in demand.passage_paths
    )
    return findings


def check_crossing_plan_info(settings: Any, state: Any, demand: Any) -> list[Finding]:
    """The statements about the plan info of a session after phase 4.

    `settings` is the `SessionSettings`, `state` the `SessionState`
    (`crossing_plan_info`, `run`), `demand` the crossing demand of the
    session's design.
    """
    info = state.crossing_plan_info
    if not info.enabled:
        return []
    if settings.crossing_mode != GUIDED_MODE:
        return _baseline_findings(info)
    findings = [
        Finding(
            "crossing_plan.event_without_job",
            ERROR,
            f"{event['edge_a']['net_name']} x {event['edge_b']['net_name']}",
        )
        for event in info.events
        if not event.get("loaded")
    ]
    guided = [
        frozenset((int(event["net_id_a"]), int(event["net_id_b"])))
        for event in info.events
        if event.get("loaded")
    ]
    if settings.crossing_guidance_net_names is None:
        findings.extend(_guidance_findings(info, guided, demand, state.run.route_jobs_by_id))
    counts: Counter[int] = Counter(net_id for pair in guided for net_id in pair)
    expected = {
        int(net_id): int(count) for net_id, count in info.expected_crossings_by_net_id.items()
    }
    if expected != dict(counts):
        findings.append(
            Finding(
                "crossing_plan.expected_counts_differ",
                ERROR,
                "expected_crossings_by_net_id",
                f"{sum(expected.values())} expected, {sum(counts.values())} in the guided pairs",
            )
        )
    return findings


def _baseline_findings(info: Any) -> list[Finding]:
    held = [
        name
        for name, value in (
            ("event_count", info.event_count),
            ("events", info.events),
            ("expected_crossings_by_net_id", info.expected_crossings_by_net_id),
            ("expected_crossings_by_net_name", info.expected_crossings_by_net_name),
            ("plan_text", info.plan_text),
            ("guidance", info.guidance),
        )
        if value
    ]
    return [Finding("crossing_plan.baseline_reads_plan", ERROR, name) for name in held]


def _guidance_findings(
    info: Any, guided: list[frozenset[int]], demand: Any, jobs_by_id: dict[int, Any]
) -> list[Finding]:
    name_of = {int(net_id): str(job.net_name) for net_id, job in jobs_by_id.items()}
    guided_names = Counter(
        frozenset(name_of.get(net_id, f"net id {net_id}") for net_id in pair) for pair in guided
    )
    planned_names = Counter(
        frozenset(edge.net_name for edge in pair) for pair in demand.invariant_pairs()
    )
    findings = [
        Finding("crossing_plan.guidance_not_the_plan", ERROR, " x ".join(sorted(pair)), detail)
        for pairs, others, detail in (
            (planned_names, guided_names, "planned, not handed to the router"),
            (guided_names, planned_names, "handed to the router, not planned"),
        )
        for pair in sorted(pairs - others, key=sorted)
    ]
    handed = info.guidance.get("planned_pair_count") if isinstance(info.guidance, dict) else None
    if handed != len(guided):
        findings.append(
            Finding(
                "crossing_plan.guidance_not_the_plan",
                ERROR,
                "planned_pair_count",
                f"{handed} handed to the router, {len(guided)} loaded event(s)",
            )
        )
    return findings


def _link_of(edge: Any) -> Link:
    return (
        str(edge.net_name),
        f"{edge.source.instance},{edge.source.port}",
        f"{edge.target.instance},{edge.target.port}",
    )


def _accounting_findings(schematic: Any, demand: Any) -> list[Finding]:
    """Every link once: confined to one band, or outside."""
    places: Counter[Link] = Counter()
    for band in demand.bands.values():
        places.update(_link_of(edge) for edge in band.left_order)
    places.update(_link_of(net.edge_key) for net in demand.outside)
    links = set(schematic_links(schematic))
    findings = [
        Finding(
            "crossing_plan.link_not_accounted",
            ERROR,
            link[0],
            f"{places[link]} time(s) in the bands and the outside list",
        )
        for link in sorted(links)
        if places[link] != 1
    ]
    findings.extend(
        Finding("crossing_plan.net_not_a_link", ERROR, link[0], f"{link[1]} - {link[2]}")
        for link in sorted(set(places) - links)
    )
    return findings


def _placed_ends(layout: Any) -> dict[tuple[str, str], tuple[float, float, float]]:
    """`(instance, port)` -> x, y and orientation (degrees) of the placed port."""
    return {
        (str(instance.name), str(port.name)): (
            float(port.center[0]),
            float(port.center[1]),
            float(port.orientation) % 360.0,
        )
        for instance in layout.insts
        for port in instance.ports
    }


def _band_findings(
    band: Any, ends: dict[tuple[str, str], tuple[float, float, float]]
) -> list[Finding]:
    name = f"band {band.index}"
    findings: list[Finding] = []
    if not band.x_min < band.x_max:
        findings.append(
            Finding("crossing_plan.band_without_width", ERROR, name, f"{band.x_min}..{band.x_max}")
        )
    left, right = list(band.left_order), list(band.right_order)
    if sorted(left, key=_link_of) != sorted(right, key=_link_of) or len(set(left)) != len(left):
        findings.append(
            Finding(
                "crossing_plan.wall_orders_differ",
                ERROR,
                name,
                f"{len(left)} net(s) on the left wall, {len(right)} on the right",
            )
        )
        return findings
    findings.extend(_wall_findings(band, ends))
    findings.extend(_pair_findings(band))
    findings.extend(_event_findings(band))
    return findings


def _wall_findings(
    band: Any, ends: dict[tuple[str, str], tuple[float, float, float]]
) -> list[Finding]:
    """Each net has an end on each wall, and the wall orders run top to bottom."""
    findings: list[Finding] = []
    heights: dict[Any, tuple[float, float]] = {}
    for edge in band.left_order:
        pair = [ends.get((ref.instance, ref.port)) for ref in (edge.source, edge.target)]
        on_left = [end for end in pair if end is not None and _points(end[2], 0.0)]
        on_right = [end for end in pair if end is not None and _points(end[2], 180.0)]
        if (
            len(on_left) != 1
            or len(on_right) != 1
            or on_left[0][0] > band.x_min + TOLERANCE_UM
            or on_right[0][0] < band.x_max - TOLERANCE_UM
        ):
            findings.append(
                Finding(
                    "crossing_plan.end_not_on_wall",
                    ERROR,
                    edge.net_name,
                    f"band {band.index} ({band.x_min:.3f}..{band.x_max:.3f} um), ends at {pair}",
                )
            )
            continue
        heights[edge] = (on_left[0][1], on_right[0][1])
    for wall, order, side in (("left", band.left_order, 0), ("right", band.right_order, 1)):
        known = [edge for edge in order if edge in heights]
        for upper, lower in pairwise(known):
            if heights[upper][side] < heights[lower][side] - TOLERANCE_UM:
                findings.append(
                    Finding(
                        "crossing_plan.wall_order_not_top_to_bottom",
                        ERROR,
                        f"band {band.index}, {wall} wall",
                        f"{upper.net_name} (y {heights[upper][side]:.3f}) before "
                        f"{lower.net_name} (y {heights[lower][side]:.3f})",
                    )
                )
    return findings


def _points(orientation: float, direction: float) -> bool:
    difference = abs(orientation - direction) % 360.0
    return min(difference, 360.0 - difference) <= ANGLE_TOLERANCE_DEG


def _pair_findings(band: Any) -> list[Finding]:
    """The invariant pairs are the inversions between the two wall orders."""
    right_rank = {edge: rank for rank, edge in enumerate(band.right_order)}
    inversions = {
        frozenset((upper, lower))
        for upper, lower in combinations(band.left_order, 2)
        if right_rank[upper] > right_rank[lower]
    }
    pairs = Counter(band.invariant_pairs)
    findings = [
        Finding("crossing_plan.pairs_not_the_inversions", ERROR, _pair_name(pair), detail)
        for some, others, detail in (
            (inversions, set(pairs), f"band {band.index}: inverted, no invariant pair"),
            (set(pairs), inversions, f"band {band.index}: invariant pair, not inverted"),
        )
        for pair in sorted(some - others, key=_pair_name)
    ]
    findings.extend(
        Finding(
            "crossing_plan.pairs_not_the_inversions",
            ERROR,
            _pair_name(pair),
            f"band {band.index}: listed {count} times",
        )
        for pair, count in pairs.items()
        if count > 1
    )
    return findings


def _event_findings(band: Any) -> list[Finding]:
    """The events are the invariant pairs and sort the left order into the right one."""
    events = band.stage.events
    name = f"band {band.index}"
    findings: list[Finding] = []
    if Counter(event.edge_pair for event in events) != Counter(band.invariant_pairs):
        findings.append(
            Finding(
                "crossing_plan.events_not_the_pairs",
                ERROR,
                name,
                f"{len(events)} event(s), {len(band.invariant_pairs)} invariant pair(s)",
            )
        )
    order = list(band.left_order)
    for event in events:
        if event.edge_a not in order or event.edge_b not in order:
            findings.append(
                Finding(
                    "crossing_plan.events_do_not_sort", ERROR, name, f"{event}: not in the band"
                )
            )
            return findings
        upper = order.index(event.edge_a)
        if upper + 1 >= len(order) or order[upper + 1] != event.edge_b:
            findings.append(
                Finding(
                    "crossing_plan.events_do_not_sort",
                    ERROR,
                    name,
                    f"{event}: the second net is not directly below the first",
                )
            )
            return findings
        order[upper], order[upper + 1] = order[upper + 1], order[upper]
    if order != list(band.right_order):
        findings.append(
            Finding(
                "crossing_plan.events_do_not_sort",
                ERROR,
                name,
                "the order after the last event is not the right wall order",
            )
        )
    return findings


def _pair_name(pair: frozenset[Any]) -> str:
    return " x ".join(sorted(edge.net_name for edge in pair))
