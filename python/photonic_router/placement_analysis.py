"""The placement analysis: what a placed design tells the router before any net is routed.

One stage, between the layout and everything that plans or routes:

```
Schematic + unrouted layout ──analyze_placement──> PlacementAnalysis
    regions           the free space as bands, passages and portals
    crossing_demand   the crossings the placement forces, band by band
```

The flow runs it once per placed design (`flow._placement_analysis_stage`)
and hands the result on: contribution 2 places its crossing tiles from
`crossing_demand.stage_schedule()`, contribution 1's routing session takes
its guidance pairs from `crossing_demand.plan`. Contribution 2 replaces the
design (tiles become instances, nets become stubs); a session that still
needs guidance then gets the analysis of that derived design.

The baseline (`lidar-pure`) never receives an analysis: its crossings are
discovered by the search and must not be informed by a plan. A routing
session entered without the flow (`route_match_and_realize` called
directly) analyses its own design with the same function.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from photonic_router.crossings.crossing_demand import CrossingDemand, build_crossing_demand
from photonic_router.crossings.region_paths import passage_load_report
from photonic_router.crossings.routing_regions import RegionGraph


@dataclass(frozen=True)
class PlacementAnalysis:
    """The result of `analyze_placement` for one placed design.

    Holds the crossing demand, which carries the region graph it was
    computed on (`regions`), the passage paths and the crossing plan.
    """

    crossing_demand: CrossingDemand

    @property
    def regions(self) -> RegionGraph:
        """The region graph of the placement: the one the crossing demand was built on."""
        return self.crossing_demand.regions

    def summary(self) -> str:
        """One line for the flow's stage output."""
        regions, demand = self.regions, self.crossing_demand
        return (
            f"{len(regions.columns)} column(s), {len(regions.bands)} band(s), "
            f"{len(regions.regions) - len(regions.bands)} passage(s); "
            f"{demand.lower_bound()} planned crossing(s), "
            f"{len(demand.outside)} net(s) outside the band model"
        )

    def passage_summary(self) -> str | None:
        """One line on the nets led through passages, or None when there are none.

        Names the vertical travel the columns add to these nets and the
        passage with the least height per path (`region_paths`).
        """
        demand = self.crossing_demand
        paths = demand.passage_paths
        if not paths:
            return None
        detour_um = sum(path.detour_um for path in paths.values())
        tightest = passage_load_report(demand.regions, paths)[0]
        return (
            f"{len(paths)} net(s) through passages, {detour_um:.1f} um detour; "
            f"tightest passage: column {tightest.key.column} slot {tightest.key.slot}, "
            f"{tightest.paths} path(s) in {tightest.height_um:.1f} um "
            f"({tightest.height_per_path_um:.1f} um each)"
        )


def analyze_placement(schematic: Any, unrouted_layout: Any) -> PlacementAnalysis:
    """Analyse a placed design: `schematic.netlist.routes` and the placed instances.

    Raises `KeyError` when a net names a port the layout does not have.
    """
    return PlacementAnalysis(crossing_demand=build_crossing_demand(schematic, unrouted_layout))
