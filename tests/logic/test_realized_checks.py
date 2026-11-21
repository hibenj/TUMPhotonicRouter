"""`photonic_router.logic_checks.realized`: each statement finds what it names.

The products are real ones: Benes 8 x 8 (48 nets, 16 crossing components)
and `cascade_8_compact` (81 nets, 25 of them with a meander), each run
through the flow until the routes are polygons. Each test breaks one thing
in a copy of the polygons, the records or the crossing plan info.
"""

from __future__ import annotations

import contextlib
import io
from dataclasses import replace
from types import SimpleNamespace

import pytest
import shapely
from photonic_router import cli
from photonic_router.logic_checks import count_by_check
from photonic_router.logic_checks.realized import (
    check_polygons,
    check_realized,
    check_verified,
    route_layer_polygons,
)
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages

pytestmark = pytest.mark.e2e


def _realized(benchmark: str):
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
        cli.main([benchmark])
    realized = seen["realized"]
    polygons = route_layer_polygons(realized.routed_layout, realized.route_layer)
    return SimpleNamespace(realized=realized, verified=seen["verified"], polygons=polygons)


@pytest.fixture(scope="module")
def benes():
    return _realized("benes_8x8")


@pytest.fixture(scope="module")
def cascade():
    return _realized("cascade_8_compact")


def _counts(product, *, polygons=None, records=None, width=None, components=None) -> dict[str, int]:
    realized = product.realized
    return count_by_check(
        check_polygons(
            product.polygons if polygons is None else polygons,
            realized.records if records is None else records,
            realized.route_width_um if width is None else width,
            _components(product) if components is None else components,
        )
    )


def _components(product) -> list:
    info = product.realized.crossing_plan_info
    return list(info.realized_crossing_components or []) if info is not None else []


def _crossing_nets(product) -> set[str]:
    return {
        str(component[key])
        for component in _components(product)
        for key in ("net_name_a", "net_name_b")
    }


def _polygons_of(product, record) -> list[int]:
    line = shapely.LineString(record.corrected_centerline_um)
    return [
        index
        for index, polygon in enumerate(product.polygons)
        if polygon.buffer(0.002).intersection(line).length > 1.0
    ]


def test_the_real_layouts_are_sound(benes, cascade) -> None:
    assert len(benes.polygons) == 48 + 2 * 16
    assert count_by_check(check_realized(benes.realized)) == {}
    assert check_verified(benes.verified) == []
    assert len(cascade.polygons) == 81
    assert _counts(cascade) == {"realized.net_with_meander": 25}


def test_an_illegal_crossing_left_by_the_repair_is_named(benes) -> None:
    left = SimpleNamespace(
        illegal_realized_crossings=[{"net_name_a": "a", "net_name_b": "b", "reason": "margin"}]
    )
    assert count_by_check(check_verified(left)) == {"realized.illegal_crossing_left": 1}


def test_polygons_and_nets_belong_to_each_other(benes) -> None:
    crossing = _crossing_nets(benes)
    record = next(r for r in benes.realized.records if r.net_name not in crossing)
    (index,) = _polygons_of(benes, record)
    without = [polygon for i, polygon in enumerate(benes.polygons) if i != index]
    assert _counts(benes, polygons=without) == {"realized.net_without_polygon": 1}
    stray = shapely.box(-5000.0, -5000.0, -4990.0, -4999.5)
    assert _counts(benes, polygons=[*benes.polygons, stray]) == {"realized.polygon_without_net": 1}
    bow_tie = shapely.Polygon([(0, 0), (10, 10), (10, 0), (0, 10)])
    assert _counts(benes, polygons=[*benes.polygons, bow_tie]) == {"realized.polygon_not_simple": 1}


def test_a_net_is_cut_once_per_crossing_and_covered_between(benes) -> None:
    crossing = _crossing_nets(benes)
    # without the components every crossing net has a polygon too many and a gap
    assert _counts(benes, components=[]) == {
        "realized.pieces_not_the_crossings": len(crossing),
        "realized.centerline_uncovered": len(crossing),
    }
    # one piece of a crossing net removed
    record = next(r for r in benes.realized.records if r.net_name in crossing)
    middle = _polygons_of(benes, record)[1]
    without = [polygon for i, polygon in enumerate(benes.polygons) if i != middle]
    counts = _counts(benes, polygons=without)
    assert counts["realized.pieces_not_the_crossings"] == 1
    assert counts["realized.centerline_uncovered"] == 1


def test_the_ports_of_a_net_lie_in_its_polygons(benes) -> None:
    record = benes.realized.records[0]
    x, y = record.target_port_center_um
    moved = [replace(record, target_port_center_um=(x, y + 5.0)), *benes.realized.records[1:]]
    assert _counts(benes, records=moved) == {"realized.port_not_in_polygon": 1}


def test_the_polygons_are_as_long_as_the_records_say(benes, cascade) -> None:
    width = benes.realized.route_width_um
    assert _counts(benes, width=2.0 * width) == {"realized.length_not_the_record": 48}
    # a meander that asks for no extra length: the polygon is longer than recorded
    records = list(cascade.realized.records)
    number = next(i for i, r in enumerate(records) if r.meander_auto_plan is not None)
    plan = {**records[number].meander_auto_plan, "requested_extra_length_um": 0.0}
    records[number] = replace(records[number], meander_auto_plan=plan)
    assert _counts(cascade, records=records) == {
        "realized.net_with_meander": 25,
        "realized.length_not_the_record": 1,
    }


def test_the_polygons_of_two_nets_do_not_overlap(benes) -> None:
    crossing = _crossing_nets(benes)
    record = next(r for r in benes.realized.records if r.net_name not in crossing)
    (index,) = _polygons_of(benes, record)
    polygons = list(benes.polygons)
    polygons[index] = polygons[index].buffer(60.0)
    counts = _counts(benes, polygons=polygons)
    assert counts["realized.nets_overlap"] >= 1
    assert counts["realized.length_not_the_record"] == 1
