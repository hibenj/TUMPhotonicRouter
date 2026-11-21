"""`photonic_router.logic_checks.routing`: each statement finds what it names.

The state is a real one: Benes 8 x 8 routed by the kernel (48 nets, 16
crossings), read after the batch. Each test breaks one thing in a copy; a
kernel route cannot be changed, so a changed route is a stand-in with the
same states, segments, cells and length.
"""

from __future__ import annotations

import contextlib
import io
from dataclasses import replace
from types import SimpleNamespace

import pytest
from photonic_router import cli
from photonic_router.logic_checks import ERROR, count_by_check
from photonic_router.logic_checks.routing import check_routing
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages

pytestmark = pytest.mark.e2e


@pytest.fixture(scope="module")
def routed():
    seen = {}

    def observer(stage: str, _settings: object, product: object) -> None:
        if stage == "routed":
            seen["state"] = product
            raise StopAfterStage

    with (
        pytest.raises(StopAfterStage),
        observe_stages(observer),
        contextlib.redirect_stdout(io.StringIO()),
    ):
        cli.main(["benes_8x8"])
    return seen["state"]


def _counts(state) -> dict[str, int]:
    findings = check_routing(state)
    assert all(finding.severity == ERROR for finding in findings)
    return count_by_check(findings)


def _with_records(state, records):
    bookkeeping = replace(state.search.bookkeeping, records_by_id=records)
    return replace(state, search=replace(state.search, bookkeeping=bookkeeping))


def _with_route(state, net_id: int, **changes: object):
    """The state with one net's route replaced by a stand-in that differs in `changes`."""
    records = dict(state.search.bookkeeping.records_by_id)
    route = records[net_id].route_obj
    stand_in = SimpleNamespace(
        states=list(route.states),
        segments=[dict(segment) for segment in route.segments],
        cells=list(route.cells),
        total_length_um=float(route.total_length_um),
    )
    for name, value in changes.items():
        setattr(stand_in, name, value)
    records[net_id] = replace(records[net_id], route_obj=stand_in)
    return _with_records(state, records)


def _with_router(state, **methods: object):
    router = state.kernel.router
    stand_in = SimpleNamespace(
        crossing_events=router.crossing_events, crossing_verdicts=router.crossing_verdicts
    )
    for name, value in methods.items():
        setattr(stand_in, name, lambda value=value: value)
    return replace(state, kernel=replace(state.kernel, router=stand_in))


def _uncrossed_net(state) -> int:
    """A net that is in no crossing."""
    crossed = {
        int(event[key])
        for event in state.kernel.router.crossing_events()
        for key in ("net_id", "partner_net_id")
    }
    return next(net_id for net_id in sorted(state.run.route_jobs_by_id) if net_id not in crossed)


def test_the_real_routes_are_sound(routed) -> None:
    assert len(routed.search.bookkeeping.records_by_id) == 48
    assert len(routed.kernel.router.crossing_events()) == 16
    assert _counts(routed) == {}
    # a stand-in with nothing changed is as sound as the route it stands for
    assert _counts(_with_route(routed, 1)) == {}


def test_the_routed_nets_are_the_jobs(routed) -> None:
    net_id = _uncrossed_net(routed)
    records = dict(routed.search.bookkeeping.records_by_id)
    stray = replace(records.pop(net_id), net_name="stray")
    assert _counts(_with_records(routed, records)) == {"routing.job_without_route": 1}
    assert _counts(_with_records(routed, {**records, net_id: stray, 999: stray})) == {
        "routing.route_without_job": 1
    }


def test_a_route_starts_and_ends_in_its_jobs_states(routed) -> None:
    net_id = _uncrossed_net(routed)
    states = list(routed.search.bookkeeping.records_by_id[net_id].route_obj.states)
    assert _counts(_with_route(routed, net_id, states=states[::-1])) == {
        "routing.route_misses_source": 1,
        "routing.route_misses_target": 1,
    }
    assert _counts(_with_route(routed, net_id, states=[])) == {"routing.route_misses_source": 1}


def test_the_segments_are_chained_and_as_long_as_they_say(routed) -> None:
    net_id = _uncrossed_net(routed)
    route = routed.search.bookkeeping.records_by_id[net_id].route_obj
    segments = [dict(segment) for segment in route.segments]
    middle = len(segments) // 2
    assert _counts(
        _with_route(routed, net_id, segments=segments[:middle] + segments[middle + 1 :])
    ) == {"routing.segments_not_chained": 1, "routing.length_not_the_segments": 1}
    straight = next(i for i, segment in enumerate(segments) if segment["kind"] == "straight")
    longer = [dict(segment) for segment in segments]
    longer[straight]["length_um"] += 1.0
    assert _counts(_with_route(routed, net_id, segments=longer)) == {
        "routing.segment_length_wrong": 1,
        "routing.length_not_the_segments": 1,
    }
    bend = next(i for i, segment in enumerate(segments) if segment["kind"] != "straight")
    shorter = [dict(segment) for segment in segments]
    shorter[bend]["length_um"] = 0.0
    assert _counts(_with_route(routed, net_id, segments=shorter)) == {
        "routing.segment_length_wrong": 1,
        "routing.length_not_the_segments": 1,
    }


def test_the_lengths_agree(routed) -> None:
    net_id = _uncrossed_net(routed)
    route = routed.search.bookkeeping.records_by_id[net_id].route_obj
    # the route's own length off: it is neither its segments' nor the recorded one
    assert _counts(
        _with_route(routed, net_id, total_length_um=float(route.total_length_um) + 1.0)
    ) == {"routing.length_not_the_segments": 2}
    records = dict(routed.search.bookkeeping.records_by_id)
    records[net_id] = replace(
        records[net_id], total_length_um=records[net_id].total_length_um + 1.0
    )
    assert _counts(_with_records(routed, records)) == {"routing.length_not_the_segments": 1}


def test_the_cells_are_a_path_through_free_or_opened_cells(routed) -> None:
    net_id = _uncrossed_net(routed)
    cells = list(routed.search.bookkeeping.records_by_id[net_id].route_obj.cells)
    middle = len(cells) // 2
    assert _counts(_with_route(routed, net_id, cells=cells[:middle] + cells[middle + 1 :])) == {
        "routing.cells_not_a_path": 1
    }
    assert _counts(_with_route(routed, net_id, cells=cells[1:])) == {"routing.cells_not_a_path": 1}
    wall = set(routed.static_cells.blocked_cells_before_port_reservations) | {tuple(cells[middle])}
    walled = replace(
        routed,
        static_cells=replace(routed.static_cells, blocked_cells_before_port_reservations=wall),
    )
    assert _counts(walled) == {"routing.route_through_static": 1}


def test_two_nets_meet_exactly_where_a_crossing_is_recorded(routed) -> None:
    events = [dict(event) for event in routed.kernel.router.crossing_events()]
    # without the recorded crossings every crossing pair meets unaccounted
    assert _counts(_with_router(routed, crossing_events=[])) == {
        "routing.nets_meet_without_crossing": 16
    }
    assert _counts(_with_router(routed, crossing_events=events[1:])) == {
        "routing.nets_meet_without_crossing": 1
    }
    # a crossing recorded between two nets that stay apart
    net_id = _uncrossed_net(routed)
    apart = {**events[0], "net_id": net_id, "partner_net_id": events[0]["net_id"]}
    counts = _counts(_with_router(routed, crossing_events=[*events, apart]))
    assert counts == {"routing.crossing_without_meeting": 1}


def test_every_recorded_crossing_is_legal_and_between_routed_nets(routed) -> None:
    verdicts = [dict(verdict) for verdict in routed.kernel.router.crossing_verdicts()]
    illegal = [{**verdicts[0], "legal": False, "reason": "made up"}, *verdicts[1:]]
    assert _counts(_with_router(routed, crossing_verdicts=illegal)) == {
        "routing.crossing_not_legal": 1
    }
    stranger = [{**verdicts[0], "partner_net_id": 999}, *verdicts[1:]]
    assert _counts(_with_router(routed, crossing_verdicts=stranger)) == {
        "routing.crossing_without_routes": 1
    }
