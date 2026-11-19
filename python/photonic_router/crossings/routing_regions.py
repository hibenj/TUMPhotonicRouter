"""The free space of a placement as a graph of routing regions.

The chip is cut along the *columns* of components (a column is a maximal set
of instances whose placed x-extents overlap or touch). Two kinds of region
are free:

- a *band*: the space between two consecutive columns, over the chip height;
- a *passage*: a y-interval of a column that none of its instances occupies
  (above the topmost instance, below the bottommost, or a gap between two),
  over the column's x-extent.

A *portal* is the boundary segment a passage shares with one of the two bands
beside its column; bands touch each other only through passages. A routed net
is a path through this graph: a net confined to one band is the path of
length one that `crossing_demand` plans today, a net that passes a column
goes band, portal, passage, portal, band.

Two nets that traverse one region cross in it iff their entry and exit
points alternate around the region's boundary (`crossing_chords`); for a band
that is the comparison of the two wall orders.

The chip height is the y-extent of the placed instances; the router's grid
reaches further, so the outermost passages are the part of the free space
the placement itself bounds. A passage spans the whole x-extent of its
column although a narrower instance leaves more room beside it: regions are
rectangles between the cuts, not the exact free polygon.
"""

from __future__ import annotations

import math
from collections.abc import Hashable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise
from typing import NamedTuple, TypeVar

TOUCH_TOLERANCE_UM = 1e-3

BAND = "band"
PASSAGE = "passage"

NetT = TypeVar("NetT", bound=Hashable)


@dataclass(frozen=True)
class Column:
    """A wall: instances whose x-extents overlap or touch, over the chip height.

    `occupied` holds the merged y-intervals of the instances, bottom to top;
    a column built without them (by hand, from x-extents alone) is a solid
    wall with no passage.
    """

    index: int
    x_min: float
    x_max: float
    instances: tuple[str, ...]
    occupied: tuple[tuple[float, float], ...] = ()


class RegionKey(NamedTuple):
    """`(BAND, i, 0)`: the band between columns i and i + 1.
    `(PASSAGE, i, k)`: the k-th free interval of column i, bottom to top."""

    kind: str
    column: int
    slot: int = 0


@dataclass(frozen=True)
class Region:
    key: RegionKey
    x_min: float
    y_min: float
    x_max: float
    y_max: float


@dataclass(frozen=True)
class Portal:
    """The segment of the line `x` between `y_min` and `y_max` that a band and a passage share."""

    band: RegionKey
    passage: RegionKey
    x: float
    y_min: float
    y_max: float

    @property
    def width(self) -> float:
        return self.y_max - self.y_min

    def other(self, key: RegionKey) -> RegionKey:
        return self.passage if key == self.band else self.band


@dataclass(frozen=True)
class RegionGraph:
    columns: tuple[Column, ...]
    regions: dict[RegionKey, Region]
    portals: tuple[Portal, ...]

    def band(self, index: int) -> Region:
        """The band between columns `index` and `index + 1`."""
        return self.regions[RegionKey(BAND, index)]

    @property
    def bands(self) -> tuple[Region, ...]:
        return tuple(region for key, region in self.regions.items() if key.kind == BAND)

    def passages(self, column: int) -> tuple[Region, ...]:
        """The free intervals of a column, bottom to top."""
        return tuple(
            region
            for key, region in self.regions.items()
            if key.kind == PASSAGE and key.column == column
        )

    def portals_of(self, key: RegionKey) -> tuple[Portal, ...]:
        return tuple(portal for portal in self.portals if key in (portal.band, portal.passage))

    def neighbors(self, key: RegionKey) -> tuple[RegionKey, ...]:
        return tuple(portal.other(key) for portal in self.portals_of(key))

    def to_text(self) -> str:
        passage_count = len(self.regions) - len(self.bands)
        lines = [
            (
                f"RegionGraph: {len(self.columns)} column(s), {len(self.bands)} band(s), "
                f"{passage_count} passage(s), {len(self.portals)} portal(s)"
            )
        ]
        for column in self.columns:
            gaps = ", ".join(
                f"y {region.y_min:.1f}..{region.y_max:.1f}"
                for region in self.passages(column.index)
            )
            lines.append(
                f"column {column.index} (x {column.x_min:.1f}..{column.x_max:.1f} um, "
                f"{len(column.instances)} instance(s)): " + (gaps or "no passage")
            )
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.to_text()


def cluster_columns(boxes: Mapping[str, tuple[float, float, float, float]]) -> tuple[Column, ...]:
    """Maximal groups of instances with overlapping or touching x-extents, left to right."""
    ordered = sorted(boxes.items(), key=lambda item: (item[1][0], item[1][2], item[0]))
    columns: list[tuple[float, float, list[str]]] = []
    for name, (x_min, _y_min, x_max, _y_max) in ordered:
        if columns and x_min <= columns[-1][1] + TOUCH_TOLERANCE_UM:
            current = columns[-1]
            columns[-1] = (current[0], max(current[1], x_max), current[2] + [name])
            continue
        columns.append((x_min, x_max, [name]))
    return tuple(
        Column(
            index=index,
            x_min=x_min,
            x_max=x_max,
            instances=tuple(names),
            occupied=_merged_intervals((boxes[name][1], boxes[name][3]) for name in names),
        )
        for index, (x_min, x_max, names) in enumerate(columns)
    )


def _merged_intervals(intervals: Iterable[tuple[float, float]]) -> tuple[tuple[float, float], ...]:
    """Overlapping or touching intervals merged, in ascending order."""
    merged: list[tuple[float, float]] = []
    for low, high in sorted(intervals):
        if merged and low <= merged[-1][1] + TOUCH_TOLERANCE_UM:
            merged[-1] = (merged[-1][0], max(merged[-1][1], high))
            continue
        merged.append((low, high))
    return tuple(merged)


def build_region_graph(columns: Sequence[Column]) -> RegionGraph:
    """Bands between consecutive columns, passages in the columns, portals between them."""
    columns = tuple(columns)
    spans = [interval for column in columns for interval in column.occupied]
    y_min = min((low for low, _high in spans), default=-math.inf)
    y_max = max((high for _low, high in spans), default=math.inf)

    regions: dict[RegionKey, Region] = {}
    portals: list[Portal] = []
    for left, right in pairwise(columns):
        key = RegionKey(BAND, left.index)
        regions[key] = Region(key, left.x_max, y_min, right.x_min, y_max)
    for column in columns:
        for slot, (low, high) in enumerate(_free_intervals(column.occupied, y_min, y_max)):
            key = RegionKey(PASSAGE, column.index, slot)
            regions[key] = Region(key, column.x_min, low, column.x_max, high)
            for band_index, x in ((column.index - 1, column.x_min), (column.index, column.x_max)):
                band = RegionKey(BAND, band_index)
                if band in regions:
                    portals.append(Portal(band, key, x, low, high))
    return RegionGraph(columns=columns, regions=regions, portals=tuple(portals))


def _free_intervals(
    occupied: Sequence[tuple[float, float]], y_min: float, y_max: float
) -> list[tuple[float, float]]:
    """The parts of `y_min..y_max` outside `occupied` (ascending, merged); none for a solid wall."""
    if not occupied:
        return []
    free: list[tuple[float, float]] = []
    cursor = y_min
    for low, high in occupied:
        if low - cursor > TOUCH_TOLERANCE_UM:
            free.append((cursor, low))
        cursor = max(cursor, high)
    if y_max - cursor > TOUCH_TOLERANCE_UM:
        free.append((cursor, y_max))
    return free


def crossing_chords(chords: Iterable[tuple[NetT, int, int]]) -> tuple[frozenset[NetT], ...]:
    """The pairs of nets that must cross inside one region.

    A chord is a net with the positions of its two ends on the region's
    boundary, counted once around it; all positions are distinct. Two chords
    cross iff their ends alternate. The pairs come in the order of the
    chords' first ends (for a band: the left wall, top to bottom).
    """
    ordered = sorted((min(a, b), max(a, b), net) for net, a, b in chords)
    pairs: list[frozenset[NetT]] = []
    for i, (_first_start, first_end, first) in enumerate(ordered):
        for second_start, second_end, second in ordered[i + 1 :]:
            if second_start < first_end < second_end:
                pairs.append(frozenset((first, second)))
    return tuple(pairs)
