"""The crossings a placement forces: bands between component columns.

`build_crossing_demand(schematic, unrouted_layout)` reads only the placed
layout and the routed links. The chip is cut into vertical *bands* whose
walls are the *columns* of components (a column is a maximal set of
instances whose placed x-extents overlap or touch, taken over the full chip
height). A terminal is the placed port centre with its orientation; it
points into the band on its right (orientation 0) or on its left (180). A
net whose two terminals point into the same band from its two walls is
*confined* to that band. Two confined nets of one band must cross iff
their terminals alternate around the band's boundary, i.e. iff their
top-to-bottom order on the left wall differs from their order on the right
wall; these are the band's *invariant pairs*, and their count over all
bands is `lower_bound()`: exact for a routing that keeps every net in its
band, a lower bound otherwise.

Per band the two wall orders go through `crossing_plan.build_crossing_plan`
(the bubble-sort sequencing, one stage per band keyed `(left column, right
column)`), so contribution 2's tiles and contribution 1's guidance see the
same `CrossingStagePlan` events in the same order as the depth/rank plan
they replaced (phase 4 slice C of the 2026-10 do-over; the equality on the
17 paper benchmarks is `tests/e2e/test_crossing_demand_equality.py`).

Nets the band model does not cover (terminals on the same wall, on
non-adjacent walls, a vertical port, a port pointing off the chip) are
listed in `outside` with a reason; they get no invariant pairs and no
events. On the 17 paper benchmarks there are none.
`CrossingDemand.require_band_layered()` refuses such a design with a
`ValueError` naming the nets and reasons; `CrossingDemand.stage_schedule()`
is the `StageSchedule` contribution 2 places its tiles from: the bands with
crossing events, left to right (phase 4 slice D).

Terminals with equal y on a wall are ordered by x, then net name, as the
depth/rank plan did; a terminal recessed from its wall face (the short MMIs
of a multiport column) is ordered by y as if it sat on the wall.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass, field
from typing import Any

from photonic_router.crossings.crossing_plan import (
    CrossingPlan,
    CrossingStagePlan,
    LaneRank,
    build_crossing_plan,
)
from photonic_router.plm.path_length_graph import PortRef, RoutedEdgeKey

ORIENTATION_TOLERANCE_DEG = 1.0
TOUCH_TOLERANCE_UM = 1e-3

SAME_WALL = "same_wall"
NON_ADJACENT_WALLS = "non_adjacent_walls"
VERTICAL_PORT = "vertical_port"
CHIP_BOUNDARY = "chip_boundary"


@dataclass(frozen=True)
class Terminal:
    """A net's end: the placed port with its orientation."""

    ref: PortRef
    x: float
    y: float
    orientation: float

    @property
    def side(self) -> str:
        """'right' (points +x), 'left' (points -x) or 'vertical'."""
        angle = self.orientation % 360.0
        if min(angle, 360.0 - angle) <= ORIENTATION_TOLERANCE_DEG:
            return "right"
        if abs(angle - 180.0) <= ORIENTATION_TOLERANCE_DEG:
            return "left"
        return "vertical"


@dataclass(frozen=True)
class Column:
    """A wall: instances whose x-extents overlap or touch, over the chip height."""

    index: int
    x_min: float
    x_max: float
    instances: tuple[str, ...]


@dataclass(frozen=True)
class Band:
    """The space between columns `index` and `index + 1` and the nets confined to it."""

    index: int
    x_min: float  # right face of the left wall
    x_max: float  # left face of the right wall
    left_order: tuple[RoutedEdgeKey, ...]  # top to bottom on the left wall
    right_order: tuple[RoutedEdgeKey, ...]  # top to bottom on the right wall
    invariant_pairs: tuple[frozenset[RoutedEdgeKey], ...]
    stage: CrossingStagePlan

    @property
    def key(self) -> tuple[int, int]:
        """The stage key of `CrossingPlan.stages`: the two wall column indices."""
        return (self.index, self.index + 1)

    @property
    def nets(self) -> tuple[RoutedEdgeKey, ...]:
        return self.left_order

    @property
    def lower_bound(self) -> int:
        return len(self.invariant_pairs)

    def counts(self) -> dict[RoutedEdgeKey, int]:
        counter: Counter[RoutedEdgeKey] = Counter()
        for pair in self.invariant_pairs:
            counter.update(pair)
        return dict(counter)

    def to_text(self, *, include_orders: bool = True) -> str:
        lines = [
            (
                f"band {self.index}->{self.index + 1} (x {self.x_min:.1f}..{self.x_max:.1f} um): "
                f"{len(self.nets)} net(s), {self.lower_bound} invariant crossing(s)"
            )
        ]
        if include_orders:
            lines.append("  left wall: " + ", ".join(edge.net_name for edge in self.left_order))
            lines.append("  right wall: " + ", ".join(edge.net_name for edge in self.right_order))
        for event in self.stage.events:
            lines.append(f"  - {event}")
        return "\n".join(lines)


@dataclass(frozen=True)
class OutsideNet:
    """A net the band model does not cover, with the reason."""

    edge_key: RoutedEdgeKey
    reason: str
    detail: str


@dataclass(frozen=True)
class StageSchedule:
    """The bands with crossing events, left to right: contribution 2's stage list.

    Each stage is a `Band`: its x-extent (`x_min`, the right face of the left
    wall column, to `x_max`, the left face of the right wall column), the wall
    orders (`left_order`, `right_order`, top to bottom), the nets confined to
    it (`nets`) and its `CrossingStagePlan` (`stage`: the events in the
    bubble-sort order of `build_crossing_plan`, keyed `key == (index,
    index + 1)`). A band without an event is not a stage: nothing is placed
    in it. Built by `CrossingDemand.stage_schedule()`; the tiles builder of
    `python/photonic_router/crossings/preplaced_crossing_grids.py` iterates over it instead of the
    plan's stage dictionary (phase 4 slice D).
    """

    stages: tuple[Band, ...]

    def __iter__(self) -> Iterator[Band]:
        return iter(self.stages)

    def __len__(self) -> int:
        return len(self.stages)

    @property
    def keys(self) -> tuple[tuple[int, int], ...]:
        return tuple(band.key for band in self.stages)

    @property
    def event_count(self) -> int:
        return sum(len(band.stage.events) for band in self.stages)

    def to_text(self, *, include_orders: bool = True) -> str:
        lines = [f"StageSchedule: {self.event_count} crossing(s) in {len(self)} stage(s)"]
        lines.extend(band.to_text(include_orders=include_orders) for band in self.stages)
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.to_text()


@dataclass(frozen=True)
class CrossingDemand:
    columns: tuple[Column, ...]
    bands: dict[int, Band]  # band index -> band; only bands with confined nets
    outside: tuple[OutsideNet, ...]
    plan: CrossingPlan = field(repr=False)
    """The bands' stages, keyed `(band index, band index + 1)`, in the shape
    contribution 2 and the guidance consume."""

    def lower_bound(self) -> int:
        """Crossings every band-confined routing of this placement realises."""
        return sum(band.lower_bound for band in self.bands.values())

    def invariant_pairs(self) -> set[frozenset[RoutedEdgeKey]]:
        return {pair for band in self.bands.values() for pair in band.invariant_pairs}

    def counts_by_net_name(self) -> dict[str, int]:
        """Unavoidable crossings per net name (nets with none are absent)."""
        counter: Counter[str] = Counter()
        for band in self.bands.values():
            for edge, count in band.counts().items():
                counter[edge.net_name] += count
        return dict(sorted(counter.items()))

    def outside_by_reason(self) -> dict[str, list[OutsideNet]]:
        grouped: dict[str, list[OutsideNet]] = {}
        for net in self.outside:
            grouped.setdefault(net.reason, []).append(net)
        return grouped

    def outside_summary(self) -> str:
        """One line naming the outside nets by reason, e.g. `3 net(s) outside the
        band model (non_adjacent_walls: n_12, n_40; vertical_port: n_7)`;
        reasons sorted, nets in netlist order."""
        groups = "; ".join(
            f"{reason}: " + ", ".join(net.edge_key.net_name for net in nets)
            for reason, nets in sorted(self.outside_by_reason().items())
        )
        return f"{len(self.outside)} net(s) outside the band model ({groups})"

    def require_band_layered(self) -> None:
        """Raise `ValueError` (the `outside_summary`) unless every net is confined to a band.

        Contribution 2 places a tile for every crossing event of a band; a
        net outside the band model has no events, so its crossings would be
        missed silently. The flow turns this error into its failure path.
        """
        if self.outside:
            raise ValueError(self.outside_summary())

    def stage_schedule(self) -> StageSchedule:
        """The bands with crossing events, left to right (see `StageSchedule`)."""
        return StageSchedule(
            tuple(band for _index, band in sorted(self.bands.items()) if band.stage.events)
        )

    @property
    def events(self):
        return self.plan.events

    def to_text(self, *, include_orders: bool = True) -> str:
        lines = [
            (
                f"CrossingDemand: {self.lower_bound()} invariant crossing(s) in "
                f"{len(self.bands)} band(s) between {len(self.columns)} column(s); "
                f"{len(self.outside)} net(s) outside the band model"
            )
        ]
        for index in sorted(self.bands):
            lines.append(self.bands[index].to_text(include_orders=include_orders))
        for reason, nets in sorted(self.outside_by_reason().items()):
            names = ", ".join(f"{net.edge_key.net_name} [{net.detail}]" for net in nets)
            lines.append(f"outside ({reason}): {names}")
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.to_text()


# --------------------------------------------------------------------------
# construction
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class _Link:
    edge_key: RoutedEdgeKey
    a: Terminal  # the declared first end (`edge_key.source`)
    b: Terminal  # the declared second end (`edge_key.target`)


def build_crossing_demand(schematic: Any, unrouted_layout: Any) -> CrossingDemand:
    """The crossing demand of a placed schematic.

    `schematic.netlist.routes` gives the links (every link is one
    `RoutedEdgeKey` in its declared order, as the route jobs name them);
    `unrouted_layout.insts` gives the placed bounding boxes and ports.
    """
    boxes, ports = _placed_geometry(unrouted_layout)
    links = _links(schematic, ports)
    columns = cluster_columns(boxes)
    return demand_from_geometry(columns, links)


def _placed_geometry(
    unrouted_layout: Any,
) -> tuple[dict[str, tuple[float, float, float, float]], dict[PortRef, Terminal]]:
    boxes: dict[str, tuple[float, float, float, float]] = {}
    ports: dict[PortRef, Terminal] = {}
    for inst in unrouted_layout.insts:
        name = str(inst.name)
        box = inst.dbbox()
        boxes[name] = (float(box.left), float(box.bottom), float(box.right), float(box.top))
        for port in inst.ports:
            center = port.center
            ref = PortRef(instance=name, port=str(port.name))
            ports[ref] = Terminal(
                ref=ref,
                x=float(center[0]),
                y=float(center[1]),
                orientation=float(port.orientation),
            )
    return boxes, ports


def _links(schematic: Any, ports: Mapping[PortRef, Terminal]) -> list[_Link]:
    links: list[_Link] = []
    for net_name, bundle in schematic.netlist.routes.items():
        for p1_spec, p2_spec in bundle.links.items():
            a = _parse_port_ref(str(p1_spec))
            b = _parse_port_ref(str(p2_spec))
            for ref in (a, b):
                if ref not in ports:
                    raise KeyError(
                        f"net {net_name}: port {ref.instance},{ref.port} is not in the layout"
                    )
            links.append(_Link(RoutedEdgeKey(str(net_name), a, b), ports[a], ports[b]))
    return links


def _parse_port_ref(spec: str) -> PortRef:
    instance, port = spec.split(",", 1)
    return PortRef(instance=instance, port=port)


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
        Column(index=index, x_min=x_min, x_max=x_max, instances=tuple(names))
        for index, (x_min, x_max, names) in enumerate(columns)
    )


def _band_of(terminal: Terminal, column_index: int, column_count: int) -> int | None:
    """Band index the terminal points into, or None for the chip boundary."""
    side = terminal.side
    if side == "right":
        band = column_index
    elif side == "left":
        band = column_index - 1
    else:
        return None
    if band < 0 or band >= column_count - 1:
        return None
    return band


def _wall_order(terminals: Iterable[tuple[RoutedEdgeKey, Terminal]]) -> tuple[RoutedEdgeKey, ...]:
    """Top to bottom; equal y by x, then net name (the depth/rank plan's order)."""
    ordered = sorted(
        ((-terminal.y, terminal.x, edge.net_name, edge) for edge, terminal in terminals),
        key=lambda item: item[:3],
    )
    return tuple(edge for _neg_y, _x, _name, edge in ordered)


def demand_from_geometry(columns: tuple[Column, ...], links: Iterable[_Link]) -> CrossingDemand:
    column_of = {name: column.index for column in columns for name in column.instances}
    confined: dict[int, list[tuple[RoutedEdgeKey, Terminal, Terminal]]] = {}
    outside: list[OutsideNet] = []

    for link in links:
        a, b = link.a, link.b
        col_a, col_b = column_of[a.ref.instance], column_of[b.ref.instance]
        if a.side == "vertical" or b.side == "vertical":
            outside.append(
                OutsideNet(
                    link.edge_key, VERTICAL_PORT, f"orientations {a.orientation}/{b.orientation}"
                )
            )
            continue
        band_a = _band_of(a, col_a, len(columns))
        band_b = _band_of(b, col_b, len(columns))
        if band_a is None or band_b is None:
            outside.append(
                OutsideNet(
                    link.edge_key,
                    CHIP_BOUNDARY,
                    f"columns {col_a}/{col_b}, a port points off the chip",
                )
            )
            continue
        if band_a != band_b:
            outside.append(
                OutsideNet(
                    link.edge_key,
                    NON_ADJACENT_WALLS,
                    f"columns {col_a}->{col_b} (skips {abs(col_a - col_b) - 1} column(s))",
                )
            )
            continue
        if a.side == b.side:
            outside.append(
                OutsideNet(
                    link.edge_key,
                    SAME_WALL,
                    f"both terminals point {a.side} in band {band_a}",
                )
            )
            continue
        left, right = (a, b) if a.side == "right" else (b, a)
        confined.setdefault(band_a, []).append((link.edge_key, left, right))

    lane_ranks: list[LaneRank] = []
    orders: dict[int, tuple[tuple[RoutedEdgeKey, ...], tuple[RoutedEdgeKey, ...]]] = {}
    for band_index, nets in confined.items():
        left_order = _wall_order((edge, left) for edge, left, _right in nets)
        right_order = _wall_order((edge, right) for edge, _left, right in nets)
        orders[band_index] = (left_order, right_order)
        right_rank = {edge: rank for rank, edge in enumerate(right_order)}
        lane_ranks.extend(
            LaneRank(
                edge_key=edge,
                source_depth=band_index,
                target_depth=band_index + 1,
                source_rank=rank,
                target_rank=right_rank[edge],
            )
            for rank, edge in enumerate(left_order)
        )
    plan = build_crossing_plan(lane_ranks)

    bands: dict[int, Band] = {}
    for band_index, (left_order, right_order) in orders.items():
        bands[band_index] = Band(
            index=band_index,
            x_min=columns[band_index].x_max,
            x_max=columns[band_index + 1].x_min,
            left_order=left_order,
            right_order=right_order,
            invariant_pairs=_alternating_pairs(left_order, right_order),
            stage=plan.stages[(band_index, band_index + 1)],
        )
    return CrossingDemand(
        columns=columns,
        bands=dict(sorted(bands.items())),
        outside=tuple(outside),
        plan=plan,
    )


def _alternating_pairs(
    left_order: tuple[RoutedEdgeKey, ...],
    right_order: tuple[RoutedEdgeKey, ...],
) -> tuple[frozenset[RoutedEdgeKey], ...]:
    """Pairs whose order on the two walls differs, in left-wall order."""
    right_rank = {edge: rank for rank, edge in enumerate(right_order)}
    pairs: list[frozenset[RoutedEdgeKey]] = []
    for i, upper in enumerate(left_order):
        for lower in left_order[i + 1 :]:
            if right_rank[upper] > right_rank[lower]:
                pairs.append(frozenset((upper, lower)))
    return tuple(pairs)
