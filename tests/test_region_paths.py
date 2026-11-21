"""`photonic_router.crossings.region_paths`: nets that pass columns go through their passages."""

from __future__ import annotations

import pytest
from photonic_router.crossings.region_paths import PathRequest, passage_loads, plan_region_paths
from photonic_router.crossings.routing_regions import (
    BAND,
    PASSAGE,
    RegionKey,
    build_region_graph,
    cluster_columns,
)

# four columns over the chip height 0..100; the two inner ones have passages:
# column 1 a gap at 40..60, column 2 below 30 and above 70
_BOXES = {
    "west": (0.0, 0.0, 10.0, 100.0),
    "low": (30.0, 0.0, 40.0, 40.0),
    "high": (30.0, 60.0, 40.0, 100.0),
    "mid": (60.0, 30.0, 70.0, 70.0),
    "east": (90.0, 0.0, 100.0, 100.0),
}
_GRAPH = build_region_graph(cluster_columns(_BOXES))
_GAP = RegionKey(PASSAGE, 1, 0)
_BELOW = RegionKey(PASSAGE, 2, 0)
_ABOVE = RegionKey(PASSAGE, 2, 1)


def _path(start_band: int, start_y: float, end_band: int, end_y: float):
    paths, blocked = plan_region_paths(
        _GRAPH, [PathRequest("n", start_band, start_y, end_band, end_y)]
    )
    assert blocked == ()
    return paths["n"]


def test_a_path_alternates_bands_and_passages() -> None:
    path = _path(0, 50.0, 1, 50.0)
    assert path.regions == (RegionKey(BAND, 0), _GAP, RegionKey(BAND, 1))
    assert path.passages == (_GAP,)
    assert path.passing_y == (50.0,)
    assert path.detour_um == 0.0


def test_a_terminal_beside_an_instance_is_led_to_the_nearest_end_of_the_gap() -> None:
    path = _path(0, 10.0, 1, 20.0)
    assert path.passing_y == (40.0,)
    assert path.vertical_um == pytest.approx(30.0 + 20.0)
    assert path.direct_vertical_um == pytest.approx(10.0)
    assert path.detour_um == pytest.approx(40.0)


def test_the_passage_with_less_vertical_travel_is_taken() -> None:
    assert _path(0, 45.0, 2, 80.0).passages == (_GAP, _ABOVE)
    assert _path(0, 45.0, 2, 10.0).passages == (_GAP, _BELOW)


def test_equal_travel_goes_to_the_lower_passage() -> None:
    path = _path(0, 50.0, 2, 50.0)
    assert path.passages == (_GAP, _BELOW)
    assert path.vertical_um == pytest.approx(40.0)


def test_a_path_runs_right_to_left_through_the_same_columns() -> None:
    path = _path(2, 80.0, 0, 45.0)
    assert path.regions == (
        RegionKey(BAND, 2),
        _ABOVE,
        RegionKey(BAND, 1),
        _GAP,
        RegionKey(BAND, 0),
    )
    assert path.vertical_um == pytest.approx(_path(0, 45.0, 2, 80.0).vertical_um)


def test_a_solid_column_blocks_the_path() -> None:
    graph = build_region_graph(cluster_columns({**_BOXES, "plug": (30.0, 40.0, 40.0, 60.0)}))
    paths, blocked = plan_region_paths(graph, [PathRequest("n", 0, 50.0, 1, 50.0)])
    assert paths == {}
    assert blocked == ("n",)


def test_a_band_that_does_not_exist_blocks_the_path() -> None:
    paths, blocked = plan_region_paths(_GRAPH, [PathRequest("n", 0, 50.0, 3, 50.0)])
    assert (paths, blocked) == ({}, ("n",))


def test_passage_loads_count_the_paths_per_passage() -> None:
    paths, _blocked = plan_region_paths(
        _GRAPH,
        [
            PathRequest("a", 0, 45.0, 2, 80.0),
            PathRequest("b", 0, 55.0, 1, 55.0),
            PathRequest("c", 1, 10.0, 2, 10.0),
        ],
    )
    assert passage_loads(paths) == {_GAP: 2, _BELOW: 1, _ABOVE: 1}
