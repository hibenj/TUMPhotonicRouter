"""`photonic_router.crossings.routing_regions`: bands, passages and portals of a placement."""

from __future__ import annotations

import importlib
import math

import pytest
from photonic_router.crossings.crossing_demand import build_crossing_demand
from photonic_router.crossings.routing_regions import (
    BAND,
    PASSAGE,
    Column,
    RegionKey,
    build_region_graph,
    cluster_columns,
    crossing_chords,
)
from photonic_router.records.layout_from_schematic import layout_from_schematic

# three columns over the chip height -10..50: a full-height wall, a column
# with a gap between its two instances, and one short instance in the middle
_BOXES = {
    "wall": (0.0, -10.0, 10.0, 50.0),
    "low": (30.0, -10.0, 40.0, 10.0),
    "high": (32.0, 25.0, 38.0, 50.0),
    "short": (60.0, 15.0, 70.0, 20.0),
}


def test_columns_carry_the_merged_y_intervals_of_their_instances() -> None:
    columns = cluster_columns(
        {**_BOXES, "touching": (30.0, 10.0, 35.0, 12.0), "inside": (31.0, 0.0, 33.0, 5.0)}
    )
    assert [column.occupied for column in columns] == [
        ((-10.0, 50.0),),
        ((-10.0, 12.0), (25.0, 50.0)),
        ((15.0, 20.0),),
    ]


def test_bands_lie_between_consecutive_columns_over_the_chip_height() -> None:
    graph = build_region_graph(cluster_columns(_BOXES))
    assert [region.key for region in graph.bands] == [RegionKey(BAND, 0), RegionKey(BAND, 1)]
    band = graph.band(1)
    assert (band.x_min, band.y_min, band.x_max, band.y_max) == (40.0, -10.0, 60.0, 50.0)


def test_passages_are_the_free_intervals_of_a_column_bottom_to_top() -> None:
    graph = build_region_graph(cluster_columns(_BOXES))
    assert graph.passages(0) == ()  # the wall spans the chip height
    (gap,) = graph.passages(1)
    assert (gap.x_min, gap.y_min, gap.x_max, gap.y_max) == (30.0, 10.0, 40.0, 25.0)
    below, above = graph.passages(2)
    assert (below.key, below.y_min, below.y_max) == (RegionKey(PASSAGE, 2, 0), -10.0, 15.0)
    assert (above.key, above.y_min, above.y_max) == (RegionKey(PASSAGE, 2, 1), 20.0, 50.0)


def test_a_portal_joins_a_passage_to_each_band_beside_its_column() -> None:
    graph = build_region_graph(cluster_columns(_BOXES))
    gap = RegionKey(PASSAGE, 1, 0)
    assert [(p.band, p.x, p.y_min, p.y_max, p.width) for p in graph.portals_of(gap)] == [
        (RegionKey(BAND, 0), 30.0, 10.0, 25.0, 15.0),
        (RegionKey(BAND, 1), 40.0, 10.0, 25.0, 15.0),
    ]
    assert graph.neighbors(gap) == (RegionKey(BAND, 0), RegionKey(BAND, 1))
    # the last column has no band on its right: one portal per passage
    assert graph.neighbors(RegionKey(PASSAGE, 2, 1)) == (RegionKey(BAND, 1),)
    assert graph.neighbors(RegionKey(BAND, 1)) == (
        gap,
        RegionKey(PASSAGE, 2, 0),
        RegionKey(PASSAGE, 2, 1),
    )
    assert graph.neighbors(RegionKey(BAND, 0)) == (gap,)
    assert "column 1 (x 30.0..40.0 um, 2 instance(s)): y 10.0..25.0" in graph.to_text()


def test_columns_without_y_intervals_are_solid_walls() -> None:
    graph = build_region_graph(
        (Column(0, 0.0, 10.0, ("l",)), Column(1, 50.0, 60.0, ("m",)), Column(2, 90.0, 95.0, ("r",)))
    )
    assert len(graph.bands) == 2 and graph.portals == ()
    assert (graph.band(0).y_min, graph.band(0).y_max) == (-math.inf, math.inf)


@pytest.mark.parametrize(
    ("chords", "expected"),
    [
        ([("a", 0, 2), ("b", 1, 3)], [{"a", "b"}]),  # ends alternate
        ([("a", 0, 3), ("b", 1, 2)], []),  # b nested in a
        ([("a", 0, 1), ("b", 2, 3)], []),  # side by side
        ([("a", 2, 0), ("b", 3, 1)], [{"a", "b"}]),  # the order of a chord's ends is free
        (
            [("c", 2, 5), ("a", 0, 3), ("b", 1, 4)],
            [{"a", "b"}, {"a", "c"}, {"b", "c"}],  # pairs in the order of the first ends
        ),
    ],
)
def test_two_chords_cross_iff_their_ends_alternate(chords, expected) -> None:
    assert list(crossing_chords(chords)) == [frozenset(pair) for pair in expected]


def test_every_planned_net_of_a_benchmark_runs_through_its_one_band() -> None:
    module = importlib.import_module("benchmarks.benes_8x8")
    schematic = module.build_schematic()
    demand = build_crossing_demand(schematic, layout_from_schematic(schematic))

    assert demand.regions.columns == demand.columns
    assert len(demand.regions.bands) == len(demand.columns) - 1
    assert set(demand.paths) == {edge for band in demand.bands.values() for edge in band.nets}
    for band in demand.bands.values():
        region = demand.regions.band(band.index)
        assert (band.x_min, band.x_max) == (region.x_min, region.x_max)
        assert all(demand.paths[edge] == (region.key,) for edge in band.nets)
