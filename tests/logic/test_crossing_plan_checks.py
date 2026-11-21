"""`photonic_router.logic_checks.crossing_plan`: each statement finds what it names.

The demand is a real one (Benes 8 x 8: 16 crossings in bands; `heater_single`:
one net outside the band model); the plan info is that of a guided run of
Benes 8 x 8 up to the static hand-off. Each test breaks one thing in a copy.
"""

from __future__ import annotations

import contextlib
import io
from dataclasses import replace
from types import SimpleNamespace

import pytest
from photonic_router import cli
from photonic_router.crossings.crossing_demand import OutsideNet
from photonic_router.logic_checks import ERROR, count_by_check
from photonic_router.logic_checks.crossing_plan import (
    check_crossing_demand,
    check_crossing_plan_info,
)
from photonic_router.placement_analysis import analyze_placement
from photonic_router.records.layout_from_schematic import layout_from_schematic
from photonic_router.records.route_keys import PortRef, RoutedEdgeKey
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages

from benchmarks.benes_8x8 import build_schematic
from benchmarks.heater_single import build_schematic as build_heater_single

pytestmark = pytest.mark.integration


def _design(schematic) -> tuple:
    layout = layout_from_schematic(schematic)
    return schematic, layout, analyze_placement(schematic, layout).crossing_demand


@pytest.fixture(scope="module")
def benes() -> tuple:
    return _design(build_schematic())


@pytest.fixture(scope="module")
def heater() -> tuple:
    return _design(build_heater_single())


def _counts(schematic, layout, demand) -> dict[str, int]:
    return count_by_check(check_crossing_demand(schematic, layout, demand))


def _with_band(demand, index: int, **changes: object):
    """The demand with one band changed; `stage_events` changes the band's stage."""
    band = demand.bands[index]
    if "stage_events" in changes:
        changes["stage"] = replace(band.stage, events=changes.pop("stage_events"))
    return replace(demand, bands={**demand.bands, index: replace(band, **changes)})


def _crossing_band(demand) -> int:
    """A band with at least two crossing events."""
    return next(index for index, band in demand.bands.items() if len(band.stage.events) >= 2)


def test_the_real_demands_are_sound(benes, heater) -> None:
    assert benes[2].lower_bound() == 16
    assert _counts(*benes) == {}
    assert _counts(*heater) == {"crossing_plan.net_outside_band_model": 1}


def test_every_link_is_in_one_band_or_outside(benes, heater) -> None:
    schematic, layout, demand = heater
    assert _counts(schematic, layout, replace(demand, outside=())) == {
        "crossing_plan.link_not_accounted": 1
    }
    schematic, layout, demand = benes
    index = _crossing_band(demand)
    confined = demand.bands[index].left_order[0]
    twice = replace(demand, outside=(OutsideNet(confined, "same_wall", "made up"),))
    counts = _counts(schematic, layout, twice)
    assert counts["crossing_plan.link_not_accounted"] == 1
    stranger = RoutedEdgeKey("no_such_net", PortRef("a", "o1"), PortRef("b", "o1"))
    counts = _counts(
        schematic, layout, replace(demand, outside=(OutsideNet(stranger, "same_wall", ""),))
    )
    assert counts["crossing_plan.net_not_a_link"] == 1


def test_a_band_has_width_and_the_same_nets_on_both_walls(benes) -> None:
    schematic, layout, demand = benes
    index = _crossing_band(demand)
    band = demand.bands[index]
    counts = _counts(schematic, layout, _with_band(demand, index, x_max=band.x_min))
    assert counts["crossing_plan.band_without_width"] == 1
    counts = _counts(schematic, layout, _with_band(demand, index, right_order=band.right_order[1:]))
    assert counts == {"crossing_plan.wall_orders_differ": 1}


def test_the_ends_of_a_confined_net_are_on_the_walls(benes) -> None:
    schematic, layout, demand = benes
    index = _crossing_band(demand)
    band = demand.bands[index]
    # a left face 5 um further left: every end of the left wall now lies in the band
    counts = _counts(schematic, layout, _with_band(demand, index, x_min=band.x_min - 5.0))
    assert counts == {"crossing_plan.end_not_on_wall": len(band.left_order)}


def test_a_wall_order_runs_top_to_bottom(benes) -> None:
    schematic, layout, demand = benes
    index = _crossing_band(demand)
    band = demand.bands[index]
    order = band.left_order
    swapped = (order[1], order[0], *order[2:])
    counts = _counts(schematic, layout, _with_band(demand, index, left_order=swapped))
    assert counts["crossing_plan.wall_order_not_top_to_bottom"] == 1


def test_the_invariant_pairs_are_the_inversions_and_the_events_are_the_pairs(benes) -> None:
    schematic, layout, demand = benes
    index = _crossing_band(demand)
    band = demand.bands[index]
    counts = _counts(
        schematic, layout, _with_band(demand, index, invariant_pairs=band.invariant_pairs[1:])
    )
    assert counts == {
        "crossing_plan.pairs_not_the_inversions": 1,
        "crossing_plan.events_not_the_pairs": 1,
    }
    counts = _counts(
        schematic,
        layout,
        _with_band(demand, index, invariant_pairs=(*band.invariant_pairs, band.invariant_pairs[0])),
    )
    assert counts["crossing_plan.pairs_not_the_inversions"] == 1


def test_the_events_sort_the_left_order_into_the_right_order(benes) -> None:
    schematic, layout, demand = benes
    index = _crossing_band(demand)
    events = demand.bands[index].stage.events
    # one event short: the pairs are not covered and the order is not reached
    counts = _counts(schematic, layout, _with_band(demand, index, stage_events=events[:-1]))
    assert counts == {
        "crossing_plan.events_not_the_pairs": 1,
        "crossing_plan.events_do_not_sort": 1,
    }
    # the upper and the lower net of an event exchanged: not a swap of "upper first"
    flipped = replace(events[0], edge_a=events[0].edge_b, edge_b=events[0].edge_a)
    counts = _counts(
        schematic, layout, _with_band(demand, index, stage_events=(flipped, *events[1:]))
    )
    assert counts == {"crossing_plan.events_do_not_sort": 1}


def test_an_outside_net_without_a_way_through_the_passages_is_noted(heater) -> None:
    schematic, layout, demand = heater
    assert len(demand.passage_paths) == 1
    assert _counts(schematic, layout, replace(demand, passage_paths={})) == {
        "crossing_plan.net_outside_band_model": 1,
        "crossing_plan.outside_net_without_path": 1,
    }


# --------------------------------------------------------------------------
# the session's plan info
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def guided():
    seen = {}

    def observer(stage: str, settings: object, product: object) -> None:
        if stage == "handoff":
            seen["session"] = (settings, product)
            raise StopAfterStage

    with (
        pytest.raises(StopAfterStage),
        observe_stages(observer),
        contextlib.redirect_stdout(io.StringIO()),
    ):
        cli.main(["benes_8x8", "--crossing-mode", "lidar-guided"])
    settings, state = seen["session"]
    return settings, state, settings.placement_analysis.crossing_demand


def _info_counts(settings, state, demand) -> dict[str, int]:
    findings = check_crossing_plan_info(settings, state, demand)
    assert all(finding.severity == ERROR for finding in findings)
    return count_by_check(findings)


def _with_info(state, **changes: object):
    return replace(state, crossing_plan_info=replace(state.crossing_plan_info, **changes))


@pytest.mark.e2e
def test_the_guided_plan_info_is_the_demand(guided) -> None:
    settings, state, demand = guided
    assert state.crossing_plan_info.guidance["planned_pair_count"] == 16
    assert _info_counts(settings, state, demand) == {}


@pytest.mark.e2e
def test_the_baseline_holds_nothing_of_a_plan(guided) -> None:
    _settings, state, demand = guided
    baseline = SimpleNamespace(crossing_mode="lidar-pure", crossing_guidance_net_names=None)
    # a guided plan info under the baseline's mode: everything it holds is named
    assert _info_counts(baseline, state, demand) == {"crossing_plan.baseline_reads_plan": 6}
    empty = _with_info(
        state,
        event_count=0,
        events=[],
        expected_crossings_by_net_id={},
        expected_crossings_by_net_name={},
        plan_text=None,
        guidance=None,
    )
    assert _info_counts(baseline, empty, demand) == {}


@pytest.mark.e2e
def test_every_planned_event_has_its_jobs_and_reaches_the_router(guided) -> None:
    settings, state, demand = guided
    info = state.crossing_plan_info
    unloaded = [{**info.events[0], "loaded": False}, *info.events[1:]]
    assert _info_counts(settings, _with_info(state, events=unloaded), demand) == {
        "crossing_plan.event_without_job": 1,
        # the pair is planned and not handed on, and 16 were said to be handed on
        "crossing_plan.guidance_not_the_plan": 2,
        "crossing_plan.expected_counts_differ": 1,
    }
    short = {**info.guidance, "planned_pair_count": 15}
    assert _info_counts(settings, _with_info(state, guidance=short), demand) == {
        "crossing_plan.guidance_not_the_plan": 1
    }


@pytest.mark.e2e
def test_the_expected_crossings_count_the_guided_pairs(guided) -> None:
    settings, state, demand = guided
    expected = dict(state.crossing_plan_info.expected_crossings_by_net_id)
    net_id = next(iter(expected))
    expected[net_id] += 1
    assert _info_counts(
        settings, _with_info(state, expected_crossings_by_net_id=expected), demand
    ) == {"crossing_plan.expected_counts_differ": 1}
