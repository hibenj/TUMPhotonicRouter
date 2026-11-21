"""Paths through the region graph for nets that pass one or more columns.

A net whose two terminals point into different bands has to pass every
column between those bands, and it can only do so through a *passage* of
each (`routing_regions`). Its `RegionPath` is the alternating sequence band,
passage, band, ..., band together with the y at which it passes each
passage.

Among all such sequences the path with the least vertical travel is taken:
the x-direction is fixed by the columns, so the vertical travel is the only
length a choice of passages changes. In a least-travel path every passing y
can be taken from the two ends of its passage and the two terminal heights
clamped into it, so the search is a shortest path over those candidates,
column by column. Ties go to the lower passage, then the lower y.

The paths are geometry only: no passage has a capacity here, and two paths
may ask for the same stretch of a passage. `passage_load_report` lists the
passages by the height each of their paths would get if they shared it
evenly; whether that is enough depends on the waveguide pitch, which this
module does not know.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Hashable, Iterable, Mapping
from dataclasses import dataclass
from typing import Generic, TypeVar

from photonic_router.crossings.routing_regions import BAND, Region, RegionGraph, RegionKey

NetT = TypeVar("NetT", bound=Hashable)


@dataclass(frozen=True)
class PathRequest(Generic[NetT]):
    """A net to be led from one band to another, with its terminal heights."""

    net: NetT
    start_band: int
    start_y: float
    end_band: int
    end_y: float


@dataclass(frozen=True)
class RegionPath(Generic[NetT]):
    """A net's way through the region graph, from its first band to its last."""

    net: NetT
    regions: tuple[RegionKey, ...]
    """Band, passage, band, ..., band."""
    passing_y: tuple[float, ...]
    """The y at which the path passes each passage of `regions`, in order."""
    vertical_um: float
    """Vertical travel from the start height over the passing heights to the end height."""
    direct_vertical_um: float
    """The height difference of the two terminals: the travel without columns in the way."""

    @property
    def detour_um(self) -> float:
        """Vertical travel the columns add."""
        return self.vertical_um - self.direct_vertical_um

    @property
    def passages(self) -> tuple[RegionKey, ...]:
        """The passages of `regions`, in order."""
        return tuple(key for key in self.regions if key.kind != BAND)


def plan_region_paths(
    regions: RegionGraph, requests: Iterable[PathRequest[NetT]]
) -> tuple[dict[NetT, RegionPath[NetT]], tuple[NetT, ...]]:
    """The least-travel path of every request, and the nets that have none.

    A net has no path when a column between its two bands is a solid wall
    (no passage) or when one of its bands does not exist.
    """
    paths: dict[NetT, RegionPath[NetT]] = {}
    blocked: list[NetT] = []
    for request in requests:
        path = _least_travel_path(regions, request)
        if path is None:
            blocked.append(request.net)
        else:
            paths[request.net] = path
    return paths, tuple(blocked)


def passage_loads(paths: Mapping[NetT, RegionPath[NetT]]) -> dict[RegionKey, int]:
    """How many paths pass each passage (passages without a path are absent)."""
    counter: Counter[RegionKey] = Counter()
    for path in paths.values():
        counter.update(path.passages)
    return dict(sorted(counter.items()))


@dataclass(frozen=True)
class PassageLoad:
    """A passage that paths go through, with the room it has for them."""

    key: RegionKey
    height_um: float
    paths: int

    @property
    def height_per_path_um(self) -> float:
        """The passage's height divided evenly among its paths."""
        return self.height_um / self.paths


def passage_load_report(
    regions: RegionGraph, paths: Mapping[NetT, RegionPath[NetT]]
) -> tuple[PassageLoad, ...]:
    """Every passage with a path, the one with the least height per path first."""
    report = [
        PassageLoad(key, regions.regions[key].y_max - regions.regions[key].y_min, count)
        for key, count in passage_loads(paths).items()
    ]
    return tuple(sorted(report, key=lambda load: (load.height_per_path_um, load.key)))


def _least_travel_path(regions: RegionGraph, request: PathRequest[NetT]) -> RegionPath[NetT] | None:
    band_count = len(regions.columns) - 1
    if not (0 <= request.start_band < band_count and 0 <= request.end_band < band_count):
        return None
    step = 1 if request.end_band > request.start_band else -1
    # Going right from band i the next column is i + 1; going left it is i.
    columns = [
        band + 1 if step == 1 else band
        for band in range(request.start_band, request.end_band, step)
    ]

    # frontier: (travel so far, passages taken, passing heights) per reachable height
    frontier: dict[float, tuple[float, tuple[RegionKey, ...], tuple[float, ...]]] = {
        request.start_y: (0.0, (), ())
    }
    for column in columns:
        reached: dict[float, tuple[float, tuple[RegionKey, ...], tuple[float, ...]]] = {}
        for passage in regions.passages(column):
            for y in _candidate_heights(passage, request):
                best = min(
                    (travel + abs(y - from_y), (*keys, passage.key), (*heights, y))
                    for from_y, (travel, keys, heights) in frontier.items()
                )
                if y not in reached or best < reached[y]:
                    reached[y] = best
        if not reached:
            return None
        frontier = reached

    travel, keys, heights = min(
        (travel + abs(request.end_y - from_y), keys, heights)
        for from_y, (travel, keys, heights) in frontier.items()
    )
    sequence: list[RegionKey] = [RegionKey(BAND, request.start_band)]
    for offset, key in enumerate(keys, start=1):
        sequence.append(key)
        sequence.append(RegionKey(BAND, request.start_band + step * offset))
    return RegionPath(
        net=request.net,
        regions=tuple(sequence),
        passing_y=heights,
        vertical_um=travel,
        direct_vertical_um=abs(request.end_y - request.start_y),
    )


def _candidate_heights(passage: Region, request: PathRequest[NetT]) -> tuple[float, ...]:
    """The passage's two ends and the two terminal heights clamped into it."""
    clamped = (
        min(max(request.start_y, passage.y_min), passage.y_max),
        min(max(request.end_y, passage.y_min), passage.y_max),
    )
    return tuple(sorted({passage.y_min, passage.y_max, *clamped}))
