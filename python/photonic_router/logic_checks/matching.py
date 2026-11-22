"""Stage "matched": are the path lengths equal where the design needs them equal?

Path-length matching reads the design as a graph along the light: a node
per instance, an edge per net from the instance the light leaves to the
one it reaches (`plm.path_length_graph`). The *arrival* at a node over one
incoming net is the path length from an input to the node's entrance over
that net: the latest arrival at the net's source, plus the length the
light travels inside the source, plus the length of the net. A *group* is
the nets into one node that has more than one; the design works when the
arrivals of a group are equal, and, with output matching, when the
arrivals at the outputs are equal (all of them, or those of one depth).

`check_matched` reads what `route_match_and_realize` hands on after the
meanders are planned (`routing.products.MatchedLengths`). It takes the
graph (which net enters which node, from which instance, and the length
inside each instance) from the analysis and recomputes every arrival from
the final lengths of the records: the routed length plus the extra length
of the record's meander plan. It does not read the arrivals, requirements
or verdicts the matching computed. Whether the analysis oriented a net
along the light is not checked here.

`check_layout_matching` makes the same statement about the drawn layout:
the length of a net is the area of its polygons divided by the route
width (`realized.polygon_lengths_um`). That measure follows the recorded
lengths to a few hundredths of a micrometre per net, so the tolerance is
wider.

Errors:

- `matching.edge_without_record`, `matching.record_without_edge`: the
  nets of the graph are not exactly the records
- `matching.graph_has_cycle`: the light would come back to a node
- `matching.extra_length_negative`: a meander plan makes a net shorter
- `matching.extra_not_the_report`: the extra length of a record's plan is
  not what the meander report booked as drawn on that net
- `matching.group_not_matched`: the arrivals over the nets into one node
  differ by more than `TOLERANCE_UM`, and the benchmark did not declare
  the group as left unmatched
- `matching.outputs_not_matched`: with output matching, the arrivals at
  the outputs of one scope differ by more than `TOLERANCE_UM`
- `realized.group_not_matched`, `realized.outputs_not_matched`: the same
  two statements on the polygons, to `LAYOUT_TOLERANCE_UM`

Notes:

- `matching.group_declared_unmatched`: a group that differs and that the
  benchmark declared (`path_length_declared_unmatched_groups`)
- `realized.matching_not_measured`: a net of the graph has no measured
  polygon length (it runs through a crossing component, whose footprint
  is not route polygon), so the layout statement is not made
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from photonic_router.logic_checks.findings import ERROR, NOTE, Finding

#: the matching's own tolerance for realized arrivals (`REALIZED_PATH_LENGTH_TOLERANCE_UM`)
TOLERANCE_UM = 1e-3
#: for lengths measured on polygons: a path of some ten nets, each to a few hundredths
LAYOUT_TOLERANCE_UM = 0.5


def final_lengths_um(records: list[Any]) -> dict[str, float]:
    """Per net name, the routed length plus the extra length of its meander plan."""
    return {str(record.net_name): _routed_um(record) + _extra_um(record) for record in records}


def _routed_um(record: Any) -> float:
    return float(record.total_length_um)


def _extra_um(record: Any) -> float:
    plan = record.meander_auto_plan
    return float(plan["requested_extra_length_um"]) if plan is not None else 0.0


def check_matched(matched: Any) -> list[Finding]:
    """The statements about a `MatchedLengths` (see the module docstring)."""
    timings = matched.analysis.node_timings
    records = {str(record.net_name): record for record in matched.records}
    edges = {
        str(edge.edge_key.net_name) for timing in timings.values() for edge in timing.incoming_edges
    }
    findings = [
        Finding("matching.edge_without_record", ERROR, name)
        for name in sorted(edges - set(records))
    ]
    findings.extend(
        Finding("matching.record_without_edge", ERROR, name)
        for name in sorted(set(records) - edges)
    )
    findings.extend(_plan_findings(matched.records, matched.meander_report))
    if edges != set(records):
        # the arrivals need a length for every net of the graph
        return findings
    findings.extend(
        arrival_findings(
            "matching",
            timings,
            final_lengths_um(matched.records),
            tolerance_um=TOLERANCE_UM,
            declared_unmatched=frozenset(matched.declared_unmatched_groups),
            output_scope=matched.output_scope,
        )
    )
    return findings


def check_layout_matching(
    matched: Any, polygon_length_by_net: Mapping[str, float]
) -> list[Finding]:
    """The arrival statements of `check_matched` on lengths measured on the polygons.

    `polygon_length_by_net` has the nets whose polygons hold their whole
    length (`realized.polygon_lengths_um`).
    """
    timings = matched.analysis.node_timings
    missing = sorted(
        {
            str(edge.edge_key.net_name)
            for timing in timings.values()
            for edge in timing.incoming_edges
        }
        - set(polygon_length_by_net)
    )
    if missing:
        return [
            Finding(
                "realized.matching_not_measured",
                NOTE,
                missing[0],
                f"{len(missing)} net(s) without a measured polygon length",
            )
        ]
    return arrival_findings(
        "realized",
        timings,
        polygon_length_by_net,
        tolerance_um=LAYOUT_TOLERANCE_UM,
        declared_unmatched=frozenset(matched.declared_unmatched_groups),
        output_scope=matched.output_scope,
    )


def _plan_findings(records: list[Any], meander_report: Any) -> list[Finding]:
    findings: list[Finding] = []
    drawn = {
        str(edge.net_name): float(extra_um)
        for edge, extra_um in meander_report.realized_extra_by_edge.items()
    }
    for record in records:
        name = str(record.net_name)
        extra_um = _extra_um(record)
        if extra_um < -TOLERANCE_UM:
            findings.append(
                Finding("matching.extra_length_negative", ERROR, name, f"{extra_um:.6f} um")
            )
        booked_um = drawn.get(name, 0.0)
        if abs(extra_um - booked_um) > TOLERANCE_UM:
            findings.append(
                Finding(
                    "matching.extra_not_the_report",
                    ERROR,
                    name,
                    f"the plan adds {extra_um:.6f} um, the report booked {booked_um:.6f} um",
                )
            )
    return findings


def arrival_findings(
    prefix: str,
    timings: Mapping[str, Any],
    length_by_net: Mapping[str, float],
    *,
    tolerance_um: float,
    declared_unmatched: frozenset[str],
    output_scope: str | None,
) -> list[Finding]:
    """The group and output statements for one set of net lengths.

    `timings` are the analysis's nodes (`NodeTiming`: type, length inside,
    incoming nets with their source instance); `output_scope` is None
    without output matching, else `all` or `depth`.
    """
    arrivals = _Arrivals(timings, length_by_net)
    for name in sorted(timings):
        if not arrivals.resolve(name):
            return [Finding(f"{prefix}.graph_has_cycle", ERROR, name)]
    findings = _group_findings(prefix, arrivals, tolerance_um, declared_unmatched)
    if output_scope is not None:
        findings.extend(_output_findings(prefix, timings, arrivals, tolerance_um, output_scope))
    return findings


def _spread(at: Mapping[str, float]) -> tuple[float, str]:
    """How far the arrivals `at` are apart, and between which two it is."""
    first, last = min(at, key=at.__getitem__), max(at, key=at.__getitem__)
    return at[last] - at[first], (
        f"{at[last] - at[first]:.6f} um between {first} ({at[first]:.6f} um) "
        f"and {last} ({at[last]:.6f} um)"
    )


def _group_findings(
    prefix: str, arrivals: _Arrivals, tolerance_um: float, declared_unmatched: frozenset[str]
) -> list[Finding]:
    findings: list[Finding] = []
    for name in sorted(arrivals.over_nets):
        over = arrivals.over_nets[name]
        if len(over) < 2:
            continue
        spread_um, detail = _spread(over)
        if spread_um <= tolerance_um:
            continue
        if name in declared_unmatched:
            findings.append(Finding(f"{prefix}.group_declared_unmatched", NOTE, name, detail))
        else:
            findings.append(Finding(f"{prefix}.group_not_matched", ERROR, name, detail))
    return findings


def _output_findings(
    prefix: str,
    timings: Mapping[str, Any],
    arrivals: _Arrivals,
    tolerance_um: float,
    output_scope: str,
) -> list[Finding]:
    scopes: dict[int, dict[str, float]] = {}
    for name, timing in timings.items():
        if str(getattr(timing.node_type, "value", timing.node_type)) != "output":
            continue
        scope = arrivals.depth[name] if output_scope == "depth" else 0
        scopes.setdefault(scope, {})[name] = arrivals.entrance[name]
    findings: list[Finding] = []
    for scope, at in sorted(scopes.items()):
        spread_um, detail = _spread(at)
        if spread_um > tolerance_um:
            subject = f"depth {scope}" if output_scope == "depth" else "all outputs"
            findings.append(Finding(f"{prefix}.outputs_not_matched", ERROR, subject, detail))
    return findings


class _Arrivals:
    """The arrivals of every node, computed along the graph from the inputs."""

    def __init__(self, timings: Mapping[str, Any], length_by_net: Mapping[str, float]) -> None:
        self._timings = timings
        self._length_by_net = length_by_net
        #: per node, the arrival over each incoming net
        self.over_nets: dict[str, dict[str, float]] = {}
        #: per node, the latest arrival at its entrance (0 without an incoming net)
        self.entrance: dict[str, float] = {}
        #: per node, the most nets on a path from an input to it
        self.depth: dict[str, int] = {}

    def resolve(self, name: str) -> bool:
        """Compute the arrivals of `name` and of everything before it; False on a cycle."""
        if name in self.entrance:
            return True
        # Iterative walk towards the inputs: a design is thousands of nodes deep.
        stack, open_nodes = [name], {name}
        while stack:
            node = stack[-1]
            sources = [str(edge.source_instance) for edge in self._timings[node].incoming_edges]
            pending = [source for source in sources if source not in self.entrance]
            if pending:
                if pending[0] in open_nodes:
                    return False
                stack.append(pending[0])
                open_nodes.add(pending[0])
                continue
            self._settle(node)
            stack.pop()
            open_nodes.discard(node)
        return True

    def _settle(self, node: str) -> None:
        over: dict[str, float] = {}
        depth = 0
        for edge in self._timings[node].incoming_edges:
            source = str(edge.source_instance)
            leaves_source_um = self.entrance[source] + float(
                self._timings[source].internal_delay_um
            )
            net = str(edge.edge_key.net_name)
            over[net] = leaves_source_um + float(self._length_by_net[net])
            depth = max(depth, self.depth[source] + 1)
        self.over_nets[node] = over
        self.entrance[node] = max(over.values(), default=0.0)
        self.depth[node] = depth
