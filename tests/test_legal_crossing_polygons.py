"""The reading of the legal crossings that both verifications share."""

from __future__ import annotations

from photonic_router.crossings.route_rust_crossing_components import (
    legal_crossing_component_footprints,
    legal_crossing_overlap_polygons,
)

_SQUARE = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]


def _crossing(**overrides: object) -> dict[str, object]:
    crossing: dict[str, object] = {
        "classification": "legal_crossing",
        "net_id_a": 7,
        "net_id_b": 3,
        "crossing_footprint_polygon_um": _SQUARE,
    }
    crossing.update(overrides)
    return crossing


def test_polygons_are_keyed_by_the_pair_with_the_smaller_id_first() -> None:
    polygons = legal_crossing_overlap_polygons([_crossing(), _crossing(net_id_a=3, net_id_b=7)])

    square = tuple((float(x), float(y)) for x, y in _SQUARE)
    assert polygons == {(3, 7): (square, square)}


def test_only_legal_crossings_with_ids_and_a_finite_polygon_count() -> None:
    skipped = [
        _crossing(classification="illegal_overlap"),
        _crossing(net_id_a=None),
        _crossing(crossing_footprint_polygon_um=_SQUARE[:2]),
        _crossing(crossing_footprint_polygon_um=[[0.0, 0.0], [1.0, float("nan")], [1.0, 1.0]]),
        _crossing(crossing_footprint_polygon_um=[[0.0, 0.0], [1.0, float("inf")], [1.0, 1.0]]),
        _crossing(crossing_footprint_polygon_um=[[0.0, 0.0], [1.0], [1.0, 1.0]]),
        _crossing(crossing_footprint_polygon_um="polygon"),
        "not a crossing",
    ]

    assert legal_crossing_overlap_polygons(skipped) == {}
    assert legal_crossing_overlap_polygons(None) == {}
    assert legal_crossing_overlap_polygons("text") == {}


def test_the_plan_object_and_its_dict_form_read_the_same() -> None:
    as_tuples = _crossing(crossing_footprint_polygon_um=tuple(tuple(p) for p in _SQUARE))

    assert legal_crossing_overlap_polygons((as_tuples,)) == legal_crossing_overlap_polygons(
        [_crossing()]
    )


def test_footprints_are_the_placed_components_when_there_are_any() -> None:
    component = {"name": "crossing_0", "bbox_um": [0.0, 0.0, 1.0, 1.0]}

    assert legal_crossing_component_footprints([component, "noise"], [_crossing()]) == (component,)


def test_footprints_fall_back_to_the_legal_intersections() -> None:
    footprints = legal_crossing_component_footprints(
        None,
        [
            _crossing(),
            _crossing(classification="illegal_overlap"),
            _crossing(crossing_footprint_polygon_um=_SQUARE[:2]),
        ],
    )

    assert footprints == (_crossing(),)
