"""`photonic_router.logic_checks.matching`: each statement finds what it names.

The product is a real one: `cascade_8_compact` (81 nets, 26 groups, 8
matched outputs), run through the flow until the routes are polygons. Each
test breaks one thing in a copy of the records, the meander report or the
measured lengths. The graph cases that no benchmark has (a cycle, outputs
at two depths) are written out as small graphs.
"""

from __future__ import annotations

import contextlib
import io
from dataclasses import replace
from types import SimpleNamespace

import pytest
from photonic_router import cli
from photonic_router.logic_checks import ERROR, count_by_check
from photonic_router.logic_checks.matching import (
    LAYOUT_TOLERANCE_UM,
    arrival_findings,
    check_layout_matching,
    check_matched,
    final_lengths_um,
)
from photonic_router.logic_checks.realized import polygon_lengths_um, route_layer_polygons
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages


@pytest.fixture(scope="module")
def cascade():
    seen = {}

    def observer(stage: str, _settings: object, product: object) -> None:
        seen[stage] = product
        if stage == "realized":
            raise StopAfterStage

    with (
        pytest.raises(StopAfterStage),
        observe_stages(observer),
        contextlib.redirect_stdout(io.StringIO()),
    ):
        cli.main(["cascade_8_compact"])
    realized = seen["realized"]
    polygons = route_layer_polygons(realized.routed_layout, realized.route_layer)
    lengths = polygon_lengths_um(polygons, realized.records, realized.route_width_um, [])
    return SimpleNamespace(matched=seen["matched"], lengths=lengths)


def _net_into(matched, *, node_type: str, incoming: int) -> tuple[str, str]:
    """A (node, net) of the graph: the first node of that type with that many incoming nets."""
    for name, timing in sorted(matched.analysis.node_timings.items()):
        if timing.node_type.value == node_type and len(timing.incoming_edges) == incoming:
            return name, str(timing.incoming_edges[0].edge_key.net_name)
    raise AssertionError(f"no {node_type} node with {incoming} incoming net(s)")


def _with_longer_net(matched, net: str, by_um: float):
    records = [
        replace(record, total_length_um=record.total_length_um + by_um)
        if record.net_name == net
        else record
        for record in matched.records
    ]
    return matched._replace(records=records)


@pytest.mark.e2e
def test_the_real_product_is_matched_in_the_records_and_in_the_layout(cascade) -> None:
    assert check_matched(cascade.matched) == []
    assert check_layout_matching(cascade.matched, cascade.lengths) == []


@pytest.mark.e2e
def test_the_layout_follows_the_records_far_inside_the_layout_tolerance(cascade) -> None:
    recorded = final_lengths_um(cascade.matched.records)
    worst = max(abs(cascade.lengths[net] - recorded[net]) for net in recorded)
    assert worst < LAYOUT_TOLERANCE_UM / 5


@pytest.mark.e2e
def test_a_graph_net_without_a_record_and_a_record_without_a_graph_net(cascade) -> None:
    matched = cascade.matched
    dropped = matched._replace(records=matched.records[1:])
    assert count_by_check(check_matched(dropped)) == {"matching.edge_without_record": 1}
    extra = replace(matched.records[0], net_name="not_in_the_graph")
    added = matched._replace(records=[*matched.records, extra])
    assert count_by_check(check_matched(added)) == {"matching.record_without_edge": 1}


@pytest.mark.e2e
def test_a_plan_that_is_not_what_the_report_booked(cascade) -> None:
    matched = cascade.matched
    report = matched.meander_report
    edge = next(iter(report.realized_extra_by_edge))
    booked = {**report.realized_extra_by_edge, edge: report.realized_extra_by_edge[edge] + 1.0}
    broken = matched._replace(meander_report=replace(report, realized_extra_by_edge=booked))
    findings = check_matched(broken)
    assert count_by_check(findings) == {"matching.extra_not_the_report": 1}
    assert findings[0].subject == edge.net_name


@pytest.mark.e2e
def test_a_plan_that_makes_a_net_shorter(cascade) -> None:
    matched = cascade.matched
    index = next(i for i, record in enumerate(matched.records) if record.meander_auto_plan)
    record = matched.records[index]
    plan = {**record.meander_auto_plan, "requested_extra_length_um": -5.0}
    records = list(matched.records)
    records[index] = replace(record, meander_auto_plan=plan)
    counts = count_by_check(check_matched(matched._replace(records=records)))
    assert counts["matching.extra_length_negative"] == 1


@pytest.mark.e2e
def test_a_group_whose_nets_arrive_apart_is_an_error_unless_declared(cascade) -> None:
    node, net = _net_into(cascade.matched, node_type="gate", incoming=2)
    broken = _with_longer_net(cascade.matched, net, 1.0)
    findings = [finding for finding in check_matched(broken) if finding.subject == node]
    assert [(finding.check, finding.severity) for finding in findings] == [
        ("matching.group_not_matched", ERROR)
    ]
    assert "1.000000 um" in findings[0].detail
    declared = broken._replace(declared_unmatched_groups=(node,))
    counts = count_by_check(
        finding for finding in check_matched(declared) if finding.subject == node
    )
    assert counts == {"matching.group_declared_unmatched": 1}


@pytest.mark.e2e
def test_a_difference_inside_the_tolerance_is_no_finding(cascade) -> None:
    _node, net = _net_into(cascade.matched, node_type="gate", incoming=2)
    assert check_matched(_with_longer_net(cascade.matched, net, 1e-4)) == []


@pytest.mark.e2e
def test_outputs_that_arrive_apart(cascade) -> None:
    _node, net = _net_into(cascade.matched, node_type="output", incoming=1)
    broken = _with_longer_net(cascade.matched, net, 1.0)
    assert count_by_check(check_matched(broken)) == {"matching.outputs_not_matched": 1}
    unmatched_outputs = broken._replace(output_scope=None)
    assert check_matched(unmatched_outputs) == []


@pytest.mark.e2e
def test_the_layout_statements_read_the_measured_lengths(cascade) -> None:
    matched = cascade.matched
    _node, net = _net_into(matched, node_type="output", incoming=1)
    longer = {**cascade.lengths, net: cascade.lengths[net] + 2 * LAYOUT_TOLERANCE_UM}
    assert count_by_check(check_layout_matching(matched, longer)) == {
        "realized.outputs_not_matched": 1
    }
    node, net = _net_into(matched, node_type="gate", incoming=2)
    longer = {**cascade.lengths, net: cascade.lengths[net] + 2 * LAYOUT_TOLERANCE_UM}
    found = [
        finding for finding in check_layout_matching(matched, longer) if finding.subject == node
    ]
    assert [finding.check for finding in found] == ["realized.group_not_matched"]
    without = {name: length for name, length in cascade.lengths.items() if name != net}
    assert count_by_check(check_layout_matching(matched, without)) == {
        "realized.matching_not_measured": 1
    }


def _node(node_type: str, delay_um: float, *incoming: tuple[str, str]) -> SimpleNamespace:
    return SimpleNamespace(
        node_type=node_type,
        internal_delay_um=delay_um,
        incoming_edges=[
            SimpleNamespace(edge_key=SimpleNamespace(net_name=net), source_instance=source)
            for net, source in incoming
        ],
    )


def _findings(timings: dict, lengths: dict, *, output_scope: str | None = None) -> dict[str, int]:
    return count_by_check(
        arrival_findings(
            "matching",
            timings,
            lengths,
            tolerance_um=1e-3,
            declared_unmatched=frozenset(),
            output_scope=output_scope,
        )
    )


def test_the_length_inside_a_node_counts_towards_the_arrival() -> None:
    timings = {
        "in": _node("input", 0.0),
        "short": _node("gate", 30.0, ("a", "in")),
        "long": _node("gate", 10.0, ("b", "in")),
        "join": _node("gate", 0.0, ("c", "short"), ("d", "long")),
    }
    # 10 + 30 + 5 against 20 + 10 + 15: equal only with the lengths inside the nodes
    assert _findings(timings, {"a": 10.0, "b": 20.0, "c": 5.0, "d": 15.0}) == {}
    assert _findings(timings, {"a": 10.0, "b": 20.0, "c": 5.0, "d": 5.0}) == {
        "matching.group_not_matched": 1
    }


def test_a_graph_in_which_the_light_comes_back_is_an_error() -> None:
    timings = {
        "a": _node("gate", 0.0, ("back", "b")),
        "b": _node("gate", 0.0, ("forth", "a")),
    }
    assert _findings(timings, {"back": 1.0, "forth": 1.0}) == {"matching.graph_has_cycle": 1}


def test_outputs_of_two_depths_are_compared_per_depth_only_in_that_scope() -> None:
    timings = {
        "in": _node("input", 0.0),
        "tap": _node("gate", 0.0, ("a", "in")),
        "early_1": _node("output", 0.0, ("b", "tap")),
        "early_2": _node("output", 0.0, ("c", "tap")),
        "stage": _node("gate", 0.0, ("d", "tap")),
        "late_1": _node("output", 0.0, ("e", "stage")),
        "late_2": _node("output", 0.0, ("f", "stage")),
    }
    lengths = {"a": 10.0, "b": 5.0, "c": 5.0, "d": 100.0, "e": 7.0, "f": 7.0}
    assert _findings(timings, lengths, output_scope="depth") == {}
    assert _findings(timings, lengths, output_scope="all") == {"matching.outputs_not_matched": 1}
    assert _findings(timings, {**lengths, "f": 8.0}, output_scope="depth") == {
        "matching.outputs_not_matched": 1
    }
