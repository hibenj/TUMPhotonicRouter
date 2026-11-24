"""Crossing-structure selector for contribution 2 (2026-09-08).

The crossing plan of the netlist graph says WHICH pairs of lanes cross in a
layer (`photonic_router.crossings.crossing_plan`). This module decides HOW that layer's
crossings are realized: it evaluates every available pre-placed alignment on
the layer's geometry, returns each candidate's feasibility (with the reason
when it is infeasible) and cost, and picks the cheapest feasible one. A layer
with no feasible alignment is reported as a router fallback; the flow then
leaves that layer's crossings to the guided search.

Candidates today:

- ``x-array``: 45-degree crossing tiles on level columns at the midpoint of
  the two swapped rows (needs every planned pair moving in opposite
  directions, no reversing lane, no dense source group, and enough band for
  its level columns). Lanes stay on their natural diagonals: the extra
  length over the octile minimum is taken as 0.
- ``column-grid``: axis-aligned "+" tiles where a lane's vertical meets
  another lane's horizontal, corners pre-wired (needs an acyclic column
  order with the 9 um corner floor and enough band for the columns). Lanes
  are Manhattan paths: the extra length over the octile minimum is
  ``min(|dy|, band) * (2 - sqrt 2)`` per moving lane.

Preference: least extra length, then fewest pre-wired corners, then fewest
tiles. The cost model is a selection rule, not an optimality proof.
"""

from __future__ import annotations

import itertools
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.config import CrossingGridConfig
from photonic_router.crossings.crossing_plan import CrossingStagePlan
from photonic_router.gds.route_gds import get_port_from_instance

if TYPE_CHECKING:  # annotation-only: `preplaced_crossing_grids` imports this module
    from photonic_router.crossings.preplaced_crossing_grids import CrossingGridGeometry

X_ARRAY = "x-array"
COLUMN_GRID = "column-grid"
ROUTER = "router"


@dataclass(frozen=True)
class StructureCost:
    """The cost of realizing one layer's crossings with one structure.

    `extra_length_um` is the lanes' total length over the octile minimum,
    `corners` the number of pre-wired corners and `tiles` the number of
    crossing tiles. `band_used_um` is reported only: for the X array it is
    the band width left after the margins, for the column grid the width its
    columns need.
    """

    extra_length_um: float
    corners: int
    tiles: int
    band_used_um: float

    def key(self) -> tuple[float, int, int]:
        """The sort key of the preference: extra length (rounded to 0.001 um), then
        corners, then tiles; the smallest key wins. `band_used_um` is not part of it."""
        return (round(self.extra_length_um, 3), self.corners, self.tiles)


@dataclass(frozen=True)
class StructureCandidate:
    """The evaluation of one structure on one layer.

    `name` is `X_ARRAY` or `COLUMN_GRID` (or `forced`, see
    `LayerDecision`). A feasible candidate carries its
    `cost`; an infeasible one carries the `reason`.
    """

    name: str
    feasible: bool
    reason: str = ""
    cost: StructureCost | None = None


@dataclass(frozen=True)
class LayerDecision:
    """The structure chosen for one layer and the candidates it was chosen from.

    `stage_key` is the key of the layer's `CrossingStagePlan`. `chosen` is
    `ROUTER` when no candidate is feasible or when the configuration forces
    the layer to the router; in the forced case `candidates` holds a single
    infeasible entry named `forced`.
    """

    stage_key: tuple[int, int]
    chosen: str  # X_ARRAY, COLUMN_GRID or ROUTER
    candidates: tuple[StructureCandidate, ...] = field(default_factory=tuple)

    def describe(self) -> str:
        """One line for the log: the layer, the chosen structure and, per candidate,
        its cost or the reason it is infeasible."""
        parts = []
        for candidate in self.candidates:
            if candidate.feasible and candidate.cost is not None:
                parts.append(
                    f"{candidate.name}: ok, +{candidate.cost.extra_length_um:.0f} um, "
                    f"{candidate.cost.corners} corners, {candidate.cost.tiles} tiles, "
                    f"{candidate.cost.band_used_um:.0f} um band"
                )
            else:
                parts.append(f"{candidate.name}: no ({candidate.reason})")
        return (
            f"layer {self.stage_key[0]}->{self.stage_key[1]}: {self.chosen} ["
            + "; ".join(parts)
            + "]"
        )


#: per net the source row and the target row in um, then the source and the
#: target x positions of the layer's nets
LayerGeometry = tuple[dict[str, float], dict[str, float], list[float], list[float]]


def _layer_geometry(
    schematic: Schematic, unrouted_layout: Component, stage_plan: CrossingStagePlan
) -> LayerGeometry:
    from photonic_router.crossings.preplaced_crossing_grids import net_endpoints

    rows: dict[str, float] = {}
    target_rows: dict[str, float] = {}
    source_xs: list[float] = []
    target_xs: list[float] = []
    for edge in stage_plan.initial_edge_order:
        p1, p2 = net_endpoints(schematic, edge.net_name)
        source = get_port_from_instance(unrouted_layout, *p1.split(","))
        target = get_port_from_instance(unrouted_layout, *p2.split(","))
        rows[edge.net_name] = float(source.dcenter[1])
        target_rows[edge.net_name] = float(target.dcenter[1])
        source_xs.append(float(source.dcenter[0]))
        target_xs.append(float(target.dcenter[0]))
    return rows, target_rows, source_xs, target_xs


def evaluate_x_array(
    schematic: Schematic,
    unrouted_layout: Component,
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry,
) -> StructureCandidate:
    """Feasibility and cost of the X array on one layer (no anchors needed)."""
    from photonic_router.crossings.preplaced_crossing_grids import lane_movement

    rows, target_rows, source_xs, target_xs = _layer_geometry(
        schematic, unrouted_layout, stage_plan
    )
    same_direction = [
        (ev.edge_a.net_name, ev.edge_b.net_name)
        for ev in stage_plan.events
        if (target_rows[ev.edge_a.net_name] - rows[ev.edge_a.net_name])
        * (target_rows[ev.edge_b.net_name] - rows[ev.edge_b.net_name])
        >= 0.0
    ]
    if same_direction:
        a, b = same_direction[0]
        return StructureCandidate(
            X_ARRAY,
            False,
            f"{len(same_direction)} pair(s) not moving in opposite directions, e.g. {a} x {b}",
        )
    by_source: dict[str, list[float]] = {}
    for edge in stage_plan.initial_edge_order:
        by_source.setdefault(edge.source.instance, []).append(rows[edge.net_name])
    for instance, group in by_source.items():
        group.sort()
        for row_a, row_b in itertools.pairwise(group):
            # Ports one to a few routing cells apart need the router's static
            # stubs, which X-array layers do not get; a 1.25 um switch pair
            # shares one cell and routes as a pair.
            if 2.0 <= row_b - row_a < 10.0:
                return StructureCandidate(
                    X_ARRAY,
                    False,
                    f"dense source group {instance} (ports {row_b - row_a:.1f} um apart)",
                )
    try:
        _movement, crossings = lane_movement(stage_plan)
    except ValueError as exc:
        return StructureCandidate(X_ARRAY, False, str(exc).split(";")[0])
    levels = max(level for level, *_ in crossings) + 1
    usable = (min(target_xs) - max(source_xs)) - 2.0 * float(geometry.band_margin_um)
    pitch = usable / float(levels)
    if pitch < float(geometry.column_pitch_um):
        return StructureCandidate(
            X_ARRAY,
            False,
            f"{levels} levels in {usable:.0f} um leave {pitch:.1f} um per level "
            f"(< {geometry.column_pitch_um:.0f})",
        )
    return StructureCandidate(
        X_ARRAY,
        True,
        cost=StructureCost(
            extra_length_um=0.0, corners=0, tiles=len(stage_plan.events), band_used_um=usable
        ),
    )


def evaluate_column_grid(
    schematic: Schematic,
    unrouted_layout: Component,
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry,
    anchors: Mapping[str, tuple[float, float]],
) -> StructureCandidate:
    """Feasibility and cost of the column grid on one layer (needs the
    router's fan-out anchors for the entry rows)."""
    from photonic_router.crossings.preplaced_crossing_grids import plan_column_grid

    try:
        probe = plan_column_grid(
            schematic, unrouted_layout, stage_plan, geometry, anchors
        ).probe_summary()
    except ValueError as exc:
        return StructureCandidate(COLUMN_GRID, False, str(exc).replace("column grid: ", "", 1))
    return StructureCandidate(
        COLUMN_GRID,
        True,
        cost=StructureCost(
            extra_length_um=float(probe["extra_length_um"]),
            corners=2 * int(probe["moving"]),
            tiles=int(probe["planned"]),
            band_used_um=float(probe["needed_um"]),
        ),
    )


def select_layer_structure(
    schematic: Schematic,
    unrouted_layout: Component,
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry,
    anchors: Mapping[str, tuple[float, float]],
    *,
    stage_key: tuple[int, int],
    config: CrossingGridConfig | None = None,
) -> LayerDecision:
    """Evaluate the X array and the column grid on the layer and pick the
    cheapest feasible one; ROUTER when none is feasible."""
    config = config if config is not None else CrossingGridConfig()
    if stage_key[0] in config.router_layers:
        return LayerDecision(
            stage_key,
            ROUTER,
            (StructureCandidate("forced", False, "PHOTONIC_ROUTER_CROSSING_GRID_ROUTER_LAYERS"),),
        )
    candidates: list[StructureCandidate] = []
    candidates.append(evaluate_x_array(schematic, unrouted_layout, stage_plan, geometry))
    candidates.append(
        evaluate_column_grid(schematic, unrouted_layout, stage_plan, geometry, anchors)
    )
    feasible = [c for c in candidates if c.feasible and c.cost is not None]
    if not feasible:
        return LayerDecision(stage_key, ROUTER, tuple(candidates))
    best = min(feasible, key=lambda c: c.cost.key())  # type: ignore[union-attr]
    return LayerDecision(stage_key, best.name, tuple(candidates))
