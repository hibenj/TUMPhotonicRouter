"""Pre-placed, placement-derived crossing tiles for permutation-network benchmarks.

For a Benes-style network the set of waveguide crossings in every interstage
layer is fully determined by the placement before routing starts (the
``CrossingDemand`` of the bands between the component columns,
`photonic_router.crossings.crossing_demand`). This module turns that precomputed
information (its ``StageSchedule``: the bands with crossing events, left to
right, each with its wall extent, wall orders and ``CrossingStagePlan``) into
placed PDK crossing
tiles -- one small tile per planned crossing (contribution 2, owner rule
2026-09-07) -- and derives a second schematic/layout pair in which those tiles
are placed as ordinary instances and every crossing lane is split into
crossing-free router nets between its tiles:

    switch output port -> first tile          (``<net>__to_grid``)
    tile -> next tile                         (``<net>__via<i>``)
    last tile -> next switch input            (``<net>__from_grid``)

The routing stage then runs with crossings disabled, because every crossing
already lives in a tile. The benchmark files themselves are never modified;
all derivation happens in memory inside the routing flow.

Tile placement
--------------

A ``CrossingStagePlan`` describes each layer as a bubble-sort network: lanes
enter in ``initial_edge_order`` (top to bottom), a sequence of *adjacent swaps*
grouped into ``level``s is applied, and lanes leave in ``final_edge_order``.
The layer's free band is measured between the lanes' own port faces
(``max`` source port x to ``min`` target port x), not from the schedule
band's wall extent: the two start at the same left face but the port band
reaches past the right wall face where a stage's target ports sit recessed
inside their column (15.9 um in three of the five multiport-MMI stages,
whose column face is set by a longer instance; equal on the Benes cells;
``tests/test_preplaced_crossing_grids.py`` pins the relation), and the
column grid centres its columns on that band. A design with nets outside
the band model never reaches this module: the flow refuses it with
``CrossingDemand.require_band_layered`` (phase 4 slice D).
Per layer, ``photonic_router.crossings.crossing_structures.select_layer_structure`` picks the
X array (the swap levels are columns spread evenly over the layer's free band,
each crossing at the midpoint of the two rows it swaps) or the column grid
(multiport MMIs), or leaves the layer to the guided router. The pre-wired
lattice and stretched-array builders that preceded the tiles, and the
experimental line placement, were removed in phase 4 slice A of the 2026-10
do-over; no paper cell used them.
"""

from __future__ import annotations

import itertools
import logging
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, NamedTuple

import gdsfactory as gf
from gdsfactory.component import Component
from gdsfactory.schematic import Instance, Net, Placement, Schematic
from gdsfactory.typings import Port

from photonic_router.config import CrossingGridConfig, RoutingConfig
from photonic_router.crossings.crossing_demand import StageSchedule
from photonic_router.crossings.crossing_plan import CrossingStagePlan
from photonic_router.gds.route_gds import get_port_from_instance
from photonic_router.records.layout_from_schematic import layout_from_schematic
from photonic_router.records.route_keys import RoutedEdgeKey
from photonic_router.routing_layers import (
    ComponentPortAccessRule,
    register_component_port_access_rule,
)

if TYPE_CHECKING:
    # the routing package imports this one; only the annotation needs the name
    from photonic_router.routing.options import SessionOptions

_LOGGER = logging.getLogger(__name__)

TO_GRID_SUFFIX = "__to_grid"
FROM_GRID_SUFFIX = "__from_grid"
# Port-to-port `connect` snaps every piece to the 1 nm database unit, so a few
# nanometres of accumulated rounding across a column is expected and harmless.
_GEOMETRY_TOLERANCE_UM = 0.02
# Tile components are content-addressed by name; the KLayout cell registry
# refuses duplicate names within one process, so a tile with an identical
# geometry is built once and reused.
_GRID_BUILD_CACHE: dict[str, CrossingGridBuildResult] = {}


@dataclass(frozen=True)
class CrossingGridGeometry:
    """Physical parameters of the pre-placed crossing tiles.

    ``bend_radius_um`` is the Euler bend radius for every bend of the corner
    tiles and static links.
    """

    bend_radius_um: float = 5.0
    # 4 um (two-cell) straight between the tiles, below what the router can
    # route as a net.
    column_pitch_um: float = 16.0
    column_lead_um: float = 14.0
    # a layer without a feasible pre-placed alignment keeps its original nets
    # and is routed by the guided search (router fallback); False makes such
    # a layer stop the run instead.
    router_fallback: bool = True
    # 9 um: crossing half-extent (4) + straight window (4) + the start of a
    # 45-degree bend; 12 um rejected one multiportmmi_32x32 pair whose
    # anchor row is 9 um from a heater row, 9 um routes and verifies clean.
    # 14 um: what the router needs to turn a lane from its column onto its
    # row right after a crossing tile (half-extent 4 + straight 4 + bend
    # radius 5, measured on the synthetic sweep: 12 um still fails). Below
    # it the tile carries the corner itself (`crossing_corner_tile_component`,
    # exact down to CORNER_TILE_MIN_UM = 9); below 9 um the pair is
    # impossible in this geometry.
    corner_margin_um: float = 14.0
    # X array: free band kept at both edges of the layer for the router's
    # port-to-tile nets (a 10 um jog with 5 um bends plus the port lanes;
    # 30 um is too tight for the router, 40 um routes); the level columns
    # share the rest.
    band_margin_um: float = 50.0
    route_width_um: float = 0.5
    cross_section: str = "strip"


@dataclass
class CrossingGridBuildResult:
    """A built crossing component with its crossing count, size in um and per-net lane data.

    The three maps are keyed by net name: the lane's length inside the component in um
    and the names of its input and output port. The single crossing tile is cached with
    empty maps.
    """

    component: Component
    crossing_count: int
    lane_length_um_by_net_name: dict[str, float]
    input_port_name_by_net_name: dict[str, str]
    output_port_name_by_net_name: dict[str, str]
    width_um: float
    height_um: float


@dataclass
class DerivedCrossingLayout:
    """The derived schematic and unrouted layout with the crossing tiles placed as instances.

    The result of the derivation described in the module docstring. Besides the two
    it carries the names of the placed tile instances, the stub nets each original net
    was split into, the length inside the tiles per original net in um, and the expected
    and the placed crossing counts.
    """

    schematic: Schematic
    unrouted_layout: Component
    grid_instance_names: list[str]
    stub_nets_by_original_net: dict[str, tuple[str, str]]
    grid_internal_length_um_by_original_net: dict[str, float]
    expected_crossing_count: int
    placed_crossing_count: int
    grid_builds: dict[str, CrossingGridBuildResult] = field(default_factory=dict)
    # tiles mode: the selector's decision per crossing layer (see
    # python/photonic_router/crossings/crossing_structures.py), in stage order
    layer_decisions: list[object] = field(default_factory=list)
    # tiles mode: original nets of the layers left to the guided router
    router_fallback_net_names: set[str] = field(default_factory=set)
    # every planned crossing of the whole plan (tiled + fallback layers)
    plan_event_count: int = 0
    # tiles mode (auto/column-grid): source instances with >= 2 crossing
    # lanes in a column-grid layer, so the routing run's fan-out access
    # gives them the same 2-port static-stub threshold this stage used for
    # its own fan-out probe. `None` when no such instance exists (or the
    # stage never ran the auto/column-grid probe).
    dense_fanout_instances: frozenset[str] | None = None


# ---------------------------------------------------------------------------
# Grid component construction
# ---------------------------------------------------------------------------


def _straight(geometry: CrossingGridGeometry, length_um: float) -> Component:
    return gf.components.straight(
        length=length_um,
        width=geometry.route_width_um,
        cross_section=geometry.cross_section,
    )


def _crossing_component() -> Component:
    return gf.components.crossing()


def _crossing_half_extent_um(crossing: Component) -> float:
    """Distance from the crossing's center to one of its ports."""
    center = crossing.dbbox().center()
    port = crossing.ports["o1"].dcenter
    return math.hypot(float(port[0]) - float(center.x), float(port[1]) - float(center.y))


def lane_movement(
    stage_plan: CrossingStagePlan,
) -> tuple[dict[str, tuple[int, int, int]], list[tuple[int, int, RoutedEdgeKey, RoutedEdgeKey]]]:
    """Simulate the swap schedule and return each lane's single movement block.

    Returns ``(movement_by_net, crossings)`` where ``movement_by_net[net] =
    (first_level, last_level, direction)`` with ``direction`` +1 for moving
    down (increasing slot index) and -1 for moving up, and ``crossings`` is a
    list of ``(level, upper_slot, upper_edge, lower_edge)`` -- the slot pair
    ``(upper_slot, upper_slot + 1)`` swapped at that level, naming which lane
    was on top before the swap. Raises if a lane reverses direction or moves
    again after pausing, since the straight-diagonal lattice cannot draw that.
    """
    order = list(stage_plan.initial_edge_order)
    events_by_level: dict[int, list] = {}
    for event in stage_plan.events:
        events_by_level.setdefault(int(event.level), []).append(event)
    level_count = (max(events_by_level) + 1) if events_by_level else 0
    movement: dict[str, tuple[int, int, int]] = {}
    crossings: list[tuple[int, int, RoutedEdgeKey, RoutedEdgeKey]] = []
    for level in range(level_count):
        moved_now: dict[str, int] = {}
        for event in events_by_level.get(level, []):
            index_a = order.index(event.edge_a)
            index_b = order.index(event.edge_b)
            if abs(index_a - index_b) != 1:
                raise ValueError(
                    "crossing event lanes are not adjacent when its level is reached: "
                    f"{event.edge_a.net_name} x {event.edge_b.net_name}"
                )
            upper_slot = min(index_a, index_b)
            upper, lower = order[upper_slot], order[upper_slot + 1]
            crossings.append((level, upper_slot, upper, lower))
            moved_now[upper.net_name] = +1
            moved_now[lower.net_name] = -1
            order[upper_slot], order[upper_slot + 1] = lower, upper
        for net_name, direction in moved_now.items():
            previous = movement.get(net_name)
            if previous is None:
                movement[net_name] = (level, level, direction)
                continue
            first, last, prev_dir = previous
            if prev_dir != direction:
                raise ValueError(
                    f"lane {net_name} reverses direction at level {level}; the "
                    "straight-diagonal crossing lattice cannot realize that"
                )
            if last != level - 1:
                raise ValueError(
                    f"lane {net_name} moves again at level {level} after pausing since "
                    f"level {last}; the straight-diagonal crossing lattice cannot realize that"
                )
            movement[net_name] = (first, level, direction)
    if tuple(order) != tuple(stage_plan.final_edge_order):
        raise RuntimeError("lane order after all levels != final_edge_order")
    return movement, crossings


def _bend_90(geometry: CrossingGridGeometry, angle: float) -> Component:
    return gf.components.bend_euler(
        angle=angle,
        radius=geometry.bend_radius_um,
        width=geometry.route_width_um,
        cross_section=geometry.cross_section,
    )


# ---------------------------------------------------------------------------
# Schematic / layout derivation
# ---------------------------------------------------------------------------


def crossing_grid_geometry_from_config(
    config: CrossingGridConfig | None = None,
) -> CrossingGridGeometry:
    """Default tile geometry, with `CrossingGridConfig` overrides for experiments.

    `config.bend_radius_um`, `config.band_margin_um`, `config.column_pitch_um`,
    `config.column_lead_um` and `config.corner_margin_um` override the
    corresponding `CrossingGridGeometry` fields when set (not `None`).
    """
    config = config if config is not None else CrossingGridConfig()
    base = CrossingGridGeometry()

    def _resolve(value: float | None, default: float) -> float:
        return default if value is None else value

    return CrossingGridGeometry(
        bend_radius_um=_resolve(config.bend_radius_um, base.bend_radius_um),
        band_margin_um=_resolve(config.band_margin_um, base.band_margin_um),
        column_pitch_um=_resolve(config.column_pitch_um, base.column_pitch_um),
        column_lead_um=_resolve(config.column_lead_um, base.column_lead_um),
        corner_margin_um=_resolve(config.corner_margin_um, base.corner_margin_um),
        route_width_um=base.route_width_um,
        cross_section=base.cross_section,
    )


def net_endpoints(schematic: Schematic, net_name: str) -> tuple[str, str]:
    """Return the two endpoints of a net's only link as strings.

    Raises ValueError if the net does not have exactly one link, KeyError if the
    schematic has no net of that name.
    """
    bundle = schematic.netlist.routes[net_name]
    links = dict(bundle.links)
    if len(links) != 1:
        raise ValueError(f"net {net_name} must have exactly one link, has {len(links)}")
    ((p1, p2),) = links.items()
    return str(p1), str(p2)


def _register_plain_cell(name: str, component: Component) -> None:
    """Register a static link straight: a grid cell with no port opening and
    no port lane -- its ports carry no net, and the router's default 8-cell
    port lane would stay reserved forever and seal the neighbouring tile
    port (multiportmmi_8x8 n_33__from_grid, 2026-09-07)."""
    _register_grid_cell(name, component, access_length_um=0.0)


def _register_grid_cell(name: str, component: Component, *, access_length_um: float = 0.0) -> None:
    # A grid port's approach is already protected by the grid's own committed
    # geometry (same reasoning as for static stubs), so it gets the smallest
    # possible access opening and no lane reservation -- otherwise the
    # reservation (+-4 cells) covers the natural row of a sibling lane that
    # crosses nothing and forces it to detour around the grid. Axis-aligned
    # "+" tiles need a short opening in front of each port: the clearance
    # ring around the tile's bounding box otherwise seals the port cell
    # (corridor diagnostics: target region size 1).
    register_component_port_access_rule(
        ComponentPortAccessRule(
            component_name_pattern=name,
            port_names=tuple(port.name for port in component.ports if port.name is not None),
            access_length_um=float(access_length_um),
            access_width_um=0.0,
        )
    )
    pdk = gf.get_active_pdk()
    if name in pdk.cells:
        return

    def factory() -> Component:
        return component

    factory.__name__ = name
    pdk.register_cells(**{name: factory})


CROSSING_TILE_PREFIX = "crossing_tile_"
CROSSING_LINK_PREFIX = "crossing_link_"
VIA_GRID_SUFFIX = "__via"
# Two consecutive elements of a lane (tiles, corners) whose ports face each
# other closer than this are joined by a static straight instead of a router
# net: the crossing rule allows footprints 0.5 um apart, the router needs a
# few free cells, and a bend body's clearance ring seals an 8 um gap.
MIN_ROUTER_GAP_UM = 10.0


def crossing_tile_component(
    geometry: CrossingGridGeometry | None = None, *, axis_aligned: bool = False
) -> Component:
    """One pre-placed crossing, centered on the origin.

    ``axis_aligned=False`` (an "X"): the PDK crossing rotated by 45 degrees,
    ports renamed by corner -- ``ul`` (upper left, faces north-west), ``ll``,
    ``ur``, ``lr``. A lane moving down enters at ``ul`` and leaves at ``lr``;
    a lane moving up enters at ``ll`` and leaves at ``ur``.
    ``axis_aligned=True`` (a "+"): the crossing as is, ports ``l``, ``r``,
    ``t``, ``b``; one lane passes horizontally ``l -> r``, the other
    vertically ``b -> t`` or ``t -> b``. The router connects to these ports
    like to any component port."""
    geometry = geometry or CrossingGridGeometry()
    crossing = _crossing_component()
    name = f"{CROSSING_TILE_PREFIX}{'plus_' if axis_aligned else ''}{crossing.name}"
    cached = _GRID_BUILD_CACHE.get(name)
    if cached is not None:
        return cached.component
    component = Component(name=name)
    ref = component.add_ref(crossing)
    if not axis_aligned:
        ref.rotate(-45.0)
    center = ref.dbbox().center()
    ref.dmove((-float(center.x), -float(center.y)))
    by_corner: dict[str, Port] = {}
    for port in ref.ports:
        x, y = float(port.dcenter[0]), float(port.dcenter[1])
        if axis_aligned:
            corner = ("l" if x < 0 else "r") if abs(x) > abs(y) else ("b" if y < 0 else "t")
        else:
            corner = ("u" if y > 0 else "l") + ("l" if x < 0 else "r")
        by_corner[corner] = port
    for corner in ("l", "r", "t", "b") if axis_aligned else ("ul", "ll", "ur", "lr"):
        component.add_port(corner, port=by_corner[corner])
    half = _crossing_half_extent_um(crossing)
    component.info["crossing_count"] = 1
    _GRID_BUILD_CACHE[name] = CrossingGridBuildResult(
        component=component,
        crossing_count=1,
        lane_length_um_by_net_name={},
        input_port_name_by_net_name={},
        output_port_name_by_net_name={},
        width_um=2.0 * half * math.sqrt(2.0),
        height_um=2.0 * half * math.sqrt(2.0),
    )
    return component


def _router_grid_origin(
    unrouted_layout: Component, router_probe_options: SessionOptions | None
) -> tuple[float, float, float] | None:
    """The routing grid's (origin_x, origin_y, cell size) the flow will use.

    Mirrors `photonic_router.static_obstacle_builder.make_grid_spec`: the
    layout's bounding box expanded by the obstacle config's security margin,
    unless the config pins a die bbox. None when no probe config is given
    (unit tests without a router configuration): tiles then stay unsnapped.
    """
    if router_probe_options is None:
        return None
    config = router_probe_options.obstacle_config
    if config is None:
        return None
    grid = float(getattr(config, "grid_size_um", 0.0) or 0.0)
    if grid <= 0.0:
        return None
    die_bbox = getattr(config, "die_bbox", None)
    if die_bbox is not None:
        return float(die_bbox[0]), float(die_bbox[1]), grid
    margin = float(getattr(config, "security_margin_um", 20.0))
    bbox = unrouted_layout.dbbox()
    return float(bbox.left) - margin, float(bbox.bottom) - margin, grid


def _port_cells_stay_inside_footprint(center: float, origin: float, grid: float) -> bool:
    """True when both 45-degree port cells on this axis coincide with the
    edge cells of the tile's rasterized footprint.

    A crossing tile's arms end at `crossing_half / sqrt 2` from the centre and
    carry the waveguide's half width beyond that. If the half width crosses a
    cell boundary that the port position does not cross, the footprint owns
    one extra cell next to the port, and the router's diagonal halo rule
    rejects the last 45-degree step into the port (seen on benes_64x64,
    2026-09-11). The condition is sub-cell parity, so it is checked per axis.
    """
    arm = _crossing_half_extent_um(_crossing_component()) / math.sqrt(2.0)
    half_width = 0.5 * float(getattr(_crossing_component().ports[0], "width", 0.5) or 0.5)

    def cell(v: float) -> int:
        return math.floor((v - origin) / grid)

    return cell(center - arm - half_width) == cell(center - arm) and cell(
        center + arm + half_width
    ) == cell(center + arm)


def _snap_x_tile_center(
    x: float, y: float, grid_origin: tuple[float, float, float] | None
) -> tuple[float, float]:
    """Snap an X-array tile centre so its port cells are the footprint's edge
    cells (see `_port_cells_stay_inside_footprint`).

    x is a free parameter of the structure (any position on the level column
    is octile-optimal), so it always goes to the nearest cell boundary, where
    the condition holds for the standard tile. y is the midpoint of the two
    swapped rows; it moves to the nearest cell boundary only when the
    condition fails there (at most half a cell)."""
    if grid_origin is None:
        return x, y
    ox, oy, g = grid_origin

    def to_boundary(v: float, origin: float) -> float:
        return origin + round((v - origin) / g) * g

    x = to_boundary(x, ox)
    if not _port_cells_stay_inside_footprint(y, oy, g):
        y = to_boundary(y, oy)
    return x, y


class _CrossingTiles(NamedTuple):
    """What `_derive_crossing_tiles` places and splits, for `derive_preplaced_crossing_layout`."""

    grid_instance_names: list[str]
    grid_builds: dict[str, CrossingGridBuildResult]
    #: original net -> its two stub nets
    stub_nets: dict[str, tuple[str, str]]
    #: original net -> the length it runs inside its grids, in um
    lengths: dict[str, float]
    segment_nets: list[Net]
    #: the original nets that were split into segments
    interstage_net_names: set[str]
    expected_crossings: int
    placed_crossings: int
    layer_decisions: list[object]
    #: the nets of the layers that stay with the router
    fallback_net_names: set[str]
    dense_fanout_instances: frozenset[str] | None


@dataclass
class _TileDerivation:
    """The placement of the crossing tiles, band by band, and what it has placed so far."""

    schematic: Schematic
    unrouted_layout: Component
    geometry: CrossingGridGeometry
    #: the schematic the tiles are added to
    derived: Schematic
    router_probe_options: SessionOptions | None
    config: CrossingGridConfig
    tile_names: list[str] = field(default_factory=list)
    builds: dict[str, CrossingGridBuildResult] = field(default_factory=dict)
    #: original net -> its two stub nets
    stub_nets: dict[str, tuple[str, str]] = field(default_factory=dict)
    #: original net -> the length it runs inside its grids, in um
    lengths: dict[str, float] = field(default_factory=dict)
    nets: list[Net] = field(default_factory=list)
    #: the original nets that were split into segments
    split: set[str] = field(default_factory=set)
    expected: int = 0
    placed: int = 0

    def add_column_grid(
        self,
        stage_plan: CrossingStagePlan,
        plus_tile: Component,
        anchors: Mapping[str, tuple[float, float]],
    ) -> None:
        """Plan one band as a column grid and place it (`plan_column_grid`, `place_column_grid`)."""
        placed = place_column_grid(
            plan_column_grid(
                self.schematic, self.unrouted_layout, stage_plan, self.geometry, anchors
            ),
            geometry=self.geometry,
            derived=self.derived,
            plus_tile=plus_tile,
            config=self.config,
        )
        self.tile_names.extend(placed.tile_names)
        self.builds.update(placed.builds)
        self.stub_nets.update(placed.stub_nets)
        self.lengths.update(placed.lengths)
        self.nets.extend(placed.nets)
        self.split.update(placed.split)
        self.expected += placed.expected_crossings
        self.placed += placed.placed_crossings

    def add_x_array(self, band_key: object, stage_plan: CrossingStagePlan, tile: Component) -> None:
        """Place one band as an X array and split its crossing lanes between the tiles.

        The swap levels are columns spread evenly over the band's free
        width minus ``band_margin_um`` on each side; the crossing of the
        lanes in slots ``s`` and ``s + 1`` sits at the midpoint of those
        lanes' source rows. Raises `ValueError` when the lanes are not
        ordered top to bottom by their source rows or the band is too
        narrow for its levels.
        """
        lanes = list(stage_plan.initial_edge_order)
        rows: list[float] = []
        source_xs: list[float] = []
        target_xs: list[float] = []
        for edge in lanes:
            p1, p2 = net_endpoints(self.schematic, edge.net_name)
            source = get_port_from_instance(self.unrouted_layout, *p1.split(","))
            target = get_port_from_instance(self.unrouted_layout, *p2.split(","))
            rows.append(float(source.dcenter[1]))
            source_xs.append(float(source.dcenter[0]))
            target_xs.append(float(target.dcenter[0]))
        for upper_row, lower_row in itertools.pairwise(rows):
            if upper_row <= lower_row:
                raise ValueError(
                    "crossing tiles need the stage's lanes ordered top to bottom by their "
                    f"source rows; stage {band_key} rows {rows}"
                )
        movement, crossings = lane_movement(stage_plan)
        level_count = max(level for level, *_ in crossings) + 1
        band_x0 = max(source_xs)
        usable = (min(target_xs) - band_x0) - 2.0 * float(self.geometry.band_margin_um)
        pitch = usable / float(level_count)
        if pitch < tile.info.get("min_pitch_um", 0.0):
            raise ValueError("crossing tiles: band too narrow")
        depth = stage_plan.source_depth
        grid_origin = _router_grid_origin(self.unrouted_layout, self.router_probe_options)
        #: (lane, level) -> the tile the lane passes at that level
        tile_of_lane_at_level: dict[tuple[str, int], str] = {}
        for level, upper_slot, upper, lower in crossings:
            name = f"{CROSSING_TILE_PREFIX}stage{depth}_l{level}_s{upper_slot}"
            x = band_x0 + float(self.geometry.band_margin_um) + (level + 0.5) * pitch
            y = 0.5 * (rows[upper_slot] + rows[upper_slot + 1])
            x, y = _snap_x_tile_center(x, y, grid_origin)
            self.derived.add_instance(
                name, Instance(component=tile.name), Placement(x=round(x, 3), y=round(y, 3))
            )
            tile_of_lane_at_level[(upper.net_name, level)] = name
            tile_of_lane_at_level[(lower.net_name, level)] = name
            self.tile_names.append(name)
            self.placed += 1
        self.expected += len(stage_plan.events)
        self.builds[f"stage{depth}"] = CrossingGridBuildResult(
            component=tile,
            crossing_count=len(crossings),
            lane_length_um_by_net_name={},
            input_port_name_by_net_name={},
            output_port_name_by_net_name={},
            width_um=usable,
            height_um=max(rows) - min(rows),
        )
        pass_length_um = 2.0 * _crossing_half_extent_um(_crossing_component())
        for edge in lanes:
            if edge.net_name in movement:  # a lane that crosses nothing stays an ordinary net
                first, last, direction = movement[edge.net_name]
                tiles = [
                    tile_of_lane_at_level[(edge.net_name, level)]
                    for level in range(first, last + 1)
                ]
                self._split_lane(edge.net_name, tiles, direction, pass_length_um)

    def _split_lane(
        self, net_name: str, tiles: list[str], direction: int, pass_length_um: float
    ) -> None:
        """Replace one lane by the nets between its port, its tiles in order, and its other port."""
        enter, leave = ("ul", "lr") if direction > 0 else ("ll", "ur")
        p1, p2 = net_endpoints(self.schematic, net_name)
        previous = p1
        names: list[str] = []
        for index, tile_name in enumerate(tiles):
            if index == 0:
                segment = f"{net_name}{TO_GRID_SUFFIX}"
            else:
                segment = f"{net_name}{VIA_GRID_SUFFIX}{index}"
            self.nets.append(Net(p1=previous, p2=f"{tile_name},{enter}", name=segment))
            names.append(segment)
            previous = f"{tile_name},{leave}"
        final = f"{net_name}{FROM_GRID_SUFFIX}"
        self.nets.append(Net(p1=previous, p2=p2, name=final))
        names.append(final)
        self.stub_nets[net_name] = (names[0], names[-1])
        self.lengths[net_name] = pass_length_um * len(tiles)
        self.split.add(net_name)


def _dense_fanout_instances(
    schematic: Schematic,
    unrouted_layout: Component,
    schedule: StageSchedule,
    geometry: CrossingGridGeometry,
) -> set[str]:
    """The source instances that get static fan-out stubs for the column grid.

    Every source instance with two or more crossing lanes in a column-grid
    layer (the router's default threshold is three), so those lanes start
    on spread rows; the router reads the same set in the routing run. A
    layer the X array can take (Benes) keeps the router's default: its
    tiles sit on the natural rows and a stub would move the lane away from
    them.
    """
    from photonic_router.crossings.crossing_structures import evaluate_x_array

    dense: set[str] = set()
    for band in schedule:
        stage_plan = band.stage
        if evaluate_x_array(schematic, unrouted_layout, stage_plan, geometry).feasible:
            continue
        count: dict[str, int] = {}
        for edge in stage_plan.initial_edge_order:
            count[edge.source.instance] = count.get(edge.source.instance, 0) + 1
        dense.update(name for name, n in count.items() if n >= 2)
    return dense


def _derive_crossing_tiles(
    schematic: Schematic,
    unrouted_layout: Component,
    schedule: StageSchedule,
    geometry: CrossingGridGeometry,
    derived: Schematic,
    router_probe_options: SessionOptions | None = None,
    config: CrossingGridConfig | None = None,
) -> _CrossingTiles:
    """Place one crossing tile per planned crossing and split every crossing
    lane into router nets between its tiles (see ``crossing_tile_component``).

    Per band of the schedule (only bands with events, left to right) the
    structure is chosen (`select_layer_structure`): the X array
    (`_TileDerivation.add_x_array`), the column grid (`add_column_grid`),
    or the band stays with the router. Raises `ValueError` when a band
    fits no structure and ``geometry.router_fallback`` is off.
    """
    from photonic_router.crossings.crossing_structures import (
        COLUMN_GRID,
        ROUTER,
        select_layer_structure,
    )
    from photonic_router.routing.api import static_fanout_anchors_um
    from photonic_router.routing.options import SessionOptions

    config = config if config is not None else CrossingGridConfig()
    tile = crossing_tile_component(geometry)
    _register_grid_cell(tile.name, tile)
    dense = _dense_fanout_instances(schematic, unrouted_layout, schedule, geometry)
    dense_fanout_instances = frozenset(dense) if dense else None
    probe_options = router_probe_options if router_probe_options is not None else SessionOptions()
    probe_config = probe_options.config or RoutingConfig()
    probe_options = replace(
        probe_options,
        config=replace(
            probe_config,
            fanout=replace(probe_config.fanout, dense_fanout_instances=dense_fanout_instances),
        ),
    )
    anchors = static_fanout_anchors_um(unrouted_layout, schematic, probe_options)
    _LOGGER.info(
        f"      - crossing tiles: {len(anchors)} static fan-out anchor(s) from the router "
        f"(dense instances: {', '.join(sorted(dense)) or 'none'})"
    )
    plus_tile = crossing_tile_component(geometry, axis_aligned=True)
    _register_grid_cell(plus_tile.name, plus_tile, access_length_um=6.0)
    derivation = _TileDerivation(
        schematic=schematic,
        unrouted_layout=unrouted_layout,
        geometry=geometry,
        derived=derived,
        router_probe_options=router_probe_options,
        config=config,
    )
    decisions: list[object] = []
    fallback_layers: list[object] = []
    fallback_nets: set[str] = set()
    for band in schedule:
        stage_plan = band.stage
        decision = select_layer_structure(
            schematic,
            unrouted_layout,
            stage_plan,
            geometry,
            anchors,
            stage_key=band.key,
            config=config,
        )
        decisions.append(decision)
        _LOGGER.info(f"      - crossing structure {decision.describe()}")
        if decision.chosen == ROUTER:
            fallback_layers.append(decision)
            fallback_nets.update(edge.net_name for edge in stage_plan.initial_edge_order)
        elif decision.chosen == COLUMN_GRID:
            derivation.add_column_grid(stage_plan, plus_tile, anchors)
        else:
            derivation.add_x_array(band.key, stage_plan, tile)
    if fallback_layers and not geometry.router_fallback:
        raise ValueError(
            "no pre-placed crossing structure fits: "
            + " | ".join(d.describe() for d in fallback_layers)  # type: ignore[attr-defined]
        )
    if fallback_layers:
        _LOGGER.info(
            f"      - crossing structure: {len(fallback_layers)} layer(s) with "
            f"{len(fallback_nets)} net(s) left to the guided router"
        )
    return _CrossingTiles(
        grid_instance_names=derivation.tile_names,
        grid_builds=derivation.builds,
        stub_nets=derivation.stub_nets,
        lengths=derivation.lengths,
        segment_nets=derivation.nets,
        interstage_net_names=derivation.split,
        expected_crossings=derivation.expected,
        placed_crossings=derivation.placed,
        layer_decisions=decisions,
        fallback_net_names=fallback_nets,
        dense_fanout_instances=dense_fanout_instances,
    )


CORNER_TILE_MIN_UM = 9.0  # crossing half-extent (4) + bend radius (5): a corner with no straight


def crossing_corner_tile_component(
    geometry: CrossingGridGeometry,
    *,
    direction: str,
    entry_d_um: float | None,
    target_d_um: float | None,
) -> Component:
    """A "+" crossing tile with one or both corners of its vertical lane
    pre-wired: the lane arrives on its entry row from the west (port ``w``,
    a 90-degree bend into the vertical, ``entry_d_um`` from the crossing
    center) and/or leaves onto its target row to the east (port ``e``,
    ``target_d_um`` from the center). ``direction`` is the vertical lane's
    travel, ``down`` or ``up``. Used where a crossing sits closer to the
    lane's own corner than the router can turn; exact down to
    ``CORNER_TILE_MIN_UM``."""
    key = (
        (
            f"{CROSSING_TILE_PREFIX}corner_{direction}_"
            f"{f'e{entry_d_um:.3f}' if entry_d_um is not None else 'x'}_"
            f"{f't{target_d_um:.3f}' if target_d_um is not None else 'x'}"
        )
        .replace(".", "p")
        .replace("-", "m")
    )
    cached = _GRID_BUILD_CACHE.get(key)
    if cached is not None:
        return cached.component
    crossing = _crossing_component()
    half = _crossing_half_extent_um(crossing)
    radius = float(geometry.bend_radius_um)
    component = Component(name=key)
    ref = component.add_ref(crossing)
    center = ref.dbbox().center()
    ref.dmove((-float(center.x), -float(center.y)))
    ports = {}
    for port in ref.ports:
        x, y = float(port.dcenter[0]), float(port.dcenter[1])
        corner = ("l" if x < 0 else "r") if abs(x) > abs(y) else ("b" if y < 0 else "t")
        ports[corner] = port
    component.add_port("l", port=ports["l"])
    component.add_port("r", port=ports["r"])
    entry_side, target_side = ("t", "b") if direction == "down" else ("b", "t")

    def _corner(side: str, d_um: float, turn_to_west: bool) -> Port:
        run = float(d_um) - half - radius
        if run < -_GEOMETRY_TOLERANCE_UM:
            raise ValueError(
                f"corner tile: {d_um:.2f} um from crossing to corner is below the "
                f"{half + radius:.1f} um a bend of radius {radius:g} needs"
            )
        open_port = ports[side]
        if run > _GEOMETRY_TOLERANCE_UM:
            straight = component.add_ref(_straight(geometry, run))
            straight.connect("o1", open_port)
            open_port = straight.ports["o2"]
        # heading of open_port is outward (north for the top side, south for
        # the bottom); a +90 turn from north goes west, from south goes east.
        heading_north = side == "t"
        wants_ccw = turn_to_west == heading_north
        bend = component.add_ref(_bend_90(geometry, 90.0 if wants_ccw else -90.0))
        bend.connect("o1", open_port)
        return bend.ports["o2"]

    if entry_d_um is not None:
        component.add_port("w", port=_corner(entry_side, entry_d_um, turn_to_west=True))
    else:
        component.add_port(entry_side, port=ports[entry_side])
    if target_d_um is not None:
        component.add_port("e", port=_corner(target_side, target_d_um, turn_to_west=False))
    else:
        component.add_port(target_side, port=ports[target_side])
    component.info["crossing_count"] = 1
    _GRID_BUILD_CACHE[key] = CrossingGridBuildResult(
        component=component,
        crossing_count=1,
        lane_length_um_by_net_name={},
        input_port_name_by_net_name={},
        output_port_name_by_net_name={},
        width_um=float(component.dbbox().width()),
        height_um=float(component.dbbox().height()),
    )
    return component


def plain_corner_component(
    geometry: CrossingGridGeometry, *, entry: bool, direction: str
) -> Component:
    """A bare 90-degree corner of a column-grid lane: entry corner (from the
    entry row heading east into the vertical, ports ``w`` and ``b``/``t``)
    or target corner (from the vertical onto the target row heading east,
    ports ``t``/``b`` and ``e``). ``direction`` is the vertical travel. The
    component origin is the port on the horizontal side for entry corners
    and the port on the vertical side for target corners."""
    key = f"{CROSSING_TILE_PREFIX}corner_only_{'entry' if entry else 'target'}_{direction}"
    cached = _GRID_BUILD_CACHE.get(key)
    if cached is not None:
        return cached.component
    component = Component(name=key)
    down = direction == "down"
    if entry:
        ref = component.add_ref(_bend_90(geometry, -90.0 if down else 90.0))
    else:
        ref = component.add_ref(_bend_90(geometry, 90.0 if down else -90.0))
        ref.rotate(-90.0 if down else 90.0)
    o1 = ref.ports["o1"].dcenter
    ref.dmove((-float(o1[0]), -float(o1[1])))
    for port in ref.ports:
        orientation = float(port.orientation) % 360.0
        name = {0.0: "e", 90.0: "t", 180.0: "w", 270.0: "b"}[round(orientation)]
        component.add_port(name, port=port)
    _GRID_BUILD_CACHE[key] = CrossingGridBuildResult(
        component=component,
        crossing_count=0,
        lane_length_um_by_net_name={},
        input_port_name_by_net_name={},
        output_port_name_by_net_name={},
        width_um=float(component.dbbox().width()),
        height_um=float(component.dbbox().height()),
    )
    return component


_COLUMN_GRID_EPS = 1e-6


def _strictly_inside(v: float, a: float, b: float) -> bool:
    """Whether `v` lies between `a` and `b`, away from both by more than the tolerance."""
    lo, hi = min(a, b), max(a, b)
    return lo + _COLUMN_GRID_EPS < v < hi - _COLUMN_GRID_EPS


@dataclass(frozen=True)
class ColumnGridPlan:
    """Where the lanes of one band run in a column grid, before anything is placed.

    Every lane is horizontal (on its entry row: the router's static fan-out
    anchor if the port has one, else the port row), then vertical in its own
    column, then horizontal on its target row. Two lanes cross where one's
    vertical meets the other's horizontal. Lanes that do not move are pure
    horizontals (no column). Lengths and positions in um.
    """

    stage_depth: int
    #: the band's nets, top to bottom as the stage plan orders them
    lanes: list[str]
    #: per lane its two ports as "instance,port"
    endpoints: dict[str, tuple[str, str]]
    #: per lane where it enters and where it ends, as (x, y)
    entry: dict[str, tuple[float, float]]
    target: dict[str, tuple[float, float]]
    #: the pairs of lanes the crossing plan lets cross
    planned: set[frozenset[str]]
    #: per lane (entry row, target row)
    span: dict[str, tuple[float, float]]
    #: the lanes whose two rows differ
    moving: list[str]
    #: the smallest distance of a crossing to its vertical lane's own corner
    #: among the pairs that fixed an order; inf when no pair did
    min_clearance: float
    column_count: int
    #: per moving lane the x of its column
    column_x: dict[str, float]
    #: the free band between the entries and the targets, less the lead on both sides
    band_x0: float
    band_x1: float
    pitch: float
    #: the width the columns take
    needed: float

    def probe_summary(self) -> dict[str, float | int]:
        """What the structure selection reads: sizes, clearance, extra length over the octile minimum."""
        band = self.band_x1 - self.band_x0
        extra = sum(
            min(abs(self.span[n][0] - self.span[n][1]), band) * (2.0 - math.sqrt(2.0))
            for n in self.moving
        )
        return {
            "min_clearance": self.min_clearance,
            "columns": self.column_count,
            "needed_um": self.needed,
            "band_um": band,
            "moving": len(self.moving),
            "lanes": len(self.lanes),
            "planned": len(self.planned),
            "extra_length_um": extra,
        }


def _lane_ends(
    schematic: Schematic,
    unrouted_layout: Component,
    stage_plan: CrossingStagePlan,
    anchors: Mapping[str, tuple[float, float]],
) -> tuple[
    dict[str, tuple[str, str]], dict[str, tuple[float, float]], dict[str, tuple[float, float]]
]:
    """Per lane its two ports and where it enters and ends: the anchor of the port, else the port."""

    def position(port_spec: str) -> tuple[float, float]:
        if port_spec in anchors:
            return anchors[port_spec]
        port = get_port_from_instance(unrouted_layout, *port_spec.split(","))
        return (float(port.dcenter[0]), float(port.dcenter[1]))

    endpoints: dict[str, tuple[str, str]] = {}
    entry: dict[str, tuple[float, float]] = {}
    target: dict[str, tuple[float, float]] = {}
    for edge in stage_plan.initial_edge_order:
        p1, p2 = net_endpoints(schematic, edge.net_name)
        endpoints[edge.net_name] = (p1, p2)
        entry[edge.net_name] = position(p1)
        target[edge.net_name] = position(p2)
    return endpoints, entry, target


def _crossing_points_if_left(
    span: Mapping[str, tuple[float, float]], a: str, b: str
) -> tuple[list[tuple[float, str]], float]:
    """Crossing rows with ``a`` left of ``b`` as ``(row, vertical lane)``,
    plus the smallest distance of those crossings to the vertical lane's
    own corners (inf when there is none)."""
    points: list[tuple[float, str]] = []
    clearance = float("inf")
    if _strictly_inside(span[b][0], *span[a]):  # a's vertical x b's entry horizontal
        row = span[b][0]
        clearance = min(clearance, abs(row - span[a][0]), abs(row - span[a][1]))
        points.append((row, a))
    if _strictly_inside(span[a][1], *span[b]):  # b's vertical x a's target horizontal
        row = span[a][1]
        clearance = min(clearance, abs(row - span[b][0]), abs(row - span[b][1]))
        points.append((row, b))
    return points, clearance


def _column_order_constraints(
    moving: list[str], planned: set[frozenset[str]], span: Mapping[str, tuple[float, float]]
) -> tuple[dict[str, set[str]], set[frozenset[str]], float]:
    """Which moving lane has to be left of which.

    Which pairs cross depends only on the left-to-right order of the
    columns: a pair whose order flips crosses once either way; a pair that
    does not flip but whose vertical spans interleave crosses in one order
    and not in the other, which fixes the order. Returns per lane the lanes
    left of it, the pairs with a fixed order and the smallest corner
    clearance among them. Raises `ValueError` for a pair that neither
    order gives its planned number of crossings.
    """
    before: dict[str, set[str]] = {n: set() for n in moving}
    tied: set[frozenset[str]] = set()
    min_clearance = float("inf")
    for a, b in itertools.combinations(moving, 2):
        need = 1 if frozenset((a, b)) in planned else 0
        pts_a, clear_a = _crossing_points_if_left(span, a, b)
        pts_b, clear_b = _crossing_points_if_left(span, b, a)
        # A corner tile (crossing with the bend attached) covers clearances
        # down to CORNER_TILE_MIN_UM, so ordering only needs that floor; the
        # router's larger corner_margin_um decides later which corners the
        # structure pre-wires.
        ok_a = len(pts_a) == need and clear_a >= CORNER_TILE_MIN_UM
        ok_b = len(pts_b) == need and clear_b >= CORNER_TILE_MIN_UM
        if ok_a and ok_b:
            continue
        if not ok_a and not ok_b:
            raise ValueError(
                f"column grid: lanes {a} and {b} need {need} crossing(s) but the "
                "horizontal-vertical-horizontal geometry gives "
                f"{len(pts_a)} at {clear_a:.1f} um / {len(pts_b)} at {clear_b:.1f} um "
                f"from a corner (corner tiles need {CORNER_TILE_MIN_UM:.0f} um)"
            )
        min_clearance = min(min_clearance, clear_a if ok_a else clear_b)
        left, right = (a, b) if ok_a else (b, a)
        before[right].add(left)
        tied.add(frozenset((a, b)))
    return before, tied, min_clearance


def _check_level_lanes(
    lanes: list[str],
    moving: list[str],
    planned: set[frozenset[str]],
    span: Mapping[str, tuple[float, float]],
) -> None:
    """Raise `ValueError` when a lane that does not move crosses a moving one it should not, or not one it should."""
    for n in lanes:
        if n in moving:
            continue
        for m in moving:
            need = 1 if frozenset((n, m)) in planned else 0
            has = 1 if _strictly_inside(span[n][0], *span[m]) else 0
            if has != need:
                raise ValueError(
                    f"column grid: level lane {n} and {m} need {need} crossing(s), "
                    f"geometry gives {has}"
                )


def _column_order(
    moving: list[str], before: Mapping[str, set[str]], span: Mapping[str, tuple[float, float]]
) -> list[str]:
    """The moving lanes left to right: the constraints first, then the widest travel, then the name."""
    order: list[str] = []
    remaining = set(moving)
    while remaining:
        ready = [n for n in remaining if before[n] <= set(order)]
        if not ready:
            raise ValueError("column grid: the column order constraints are cyclic")
        ready.sort(key=lambda n: (-abs(span[n][0] - span[n][1]), n))
        order.append(ready[0])
        remaining.remove(ready[0])
    return order


def _columns(
    order: list[str],
    before: Mapping[str, set[str]],
    tied: set[frozenset[str]],
    span: Mapping[str, tuple[float, float]],
    geometry: CrossingGridGeometry,
) -> tuple[list[list[str]], dict[str, int]]:
    """The lanes of each column and the column of each lane.

    Lanes with disjoint spans and no constraint between them share a
    column.
    """
    columns: list[list[str]] = []
    column_of: dict[str, int] = {}
    # Shared column: spans apart by more than a bare corner body plus
    # clearance (a corner bend of radius 5 plus the router's 2 um ring
    # on each side), so one lane's corner never sits on another
    # lane's vertical run.
    share_gap = float(geometry.bend_radius_um) * 2.0 + 6.0
    for n in order:
        lo, hi = min(span[n]), max(span[n])
        # A shared column must lie strictly right of every column a lane
        # that has to be left of n occupies; otherwise sharing would undo
        # the order (an unplanned crossing on the entry row, seen on
        # multiportmmi_16x16 stage 11: n_133 joined a column left of n_132).
        min_index = max((column_of[m] + 1 for m in before[n] if m in column_of), default=0)
        for index, column in enumerate(columns):
            if index < min_index:
                continue
            if all(
                (max(span[m]) < lo - share_gap or min(span[m]) > hi + share_gap)
                and frozenset((n, m)) not in tied
                for m in column
            ):
                column.append(n)
                column_of[n] = index
                break
        else:
            columns.append([n])
            column_of[n] = len(columns) - 1
    return columns, column_of


def plan_column_grid(
    schematic: Schematic,
    unrouted_layout: Component,
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry,
    anchors: Mapping[str, tuple[float, float]],
) -> ColumnGridPlan:
    """Plan the column grid of one band: axis-aligned "+" crossings only.

    The order of the columns is built from the constraints of
    `_column_order_constraints` (widest travel first as the tiebreak).
    Nothing is placed. Raises `ValueError` when the band cannot be a
    column grid: a pair of lanes does not get its planned crossings, the
    constraints are cyclic or the columns do not fit the band.
    """
    lanes = [edge.net_name for edge in stage_plan.initial_edge_order]
    endpoints, entry, target = _lane_ends(schematic, unrouted_layout, stage_plan, anchors)
    planned = {frozenset((ev.edge_a.net_name, ev.edge_b.net_name)) for ev in stage_plan.events}
    span = {n: (entry[n][1], target[n][1]) for n in lanes}
    moving = [n for n in lanes if abs(span[n][0] - span[n][1]) > _COLUMN_GRID_EPS]
    before, tied, min_clearance = _column_order_constraints(moving, planned, span)
    _check_level_lanes(lanes, moving, planned, span)
    columns, column_of = _columns(_column_order(moving, before, span), before, tied, span, geometry)
    band_x0 = max(entry[n][0] for n in lanes) + float(geometry.column_lead_um)
    band_x1 = min(target[n][0] for n in lanes) - float(geometry.column_lead_um)
    pitch = float(geometry.column_pitch_um)
    needed = (len(columns) - 1) * pitch if columns else 0.0
    if needed > band_x1 - band_x0 + _COLUMN_GRID_EPS:
        raise ValueError(
            f"column grid: stage {stage_plan.source_depth} needs {len(columns)} columns "
            f"({needed:.1f} um) but the band leaves {band_x1 - band_x0:.1f} um"
        )
    x_start = 0.5 * (band_x0 + band_x1 - needed)
    return ColumnGridPlan(
        stage_depth=stage_plan.source_depth,
        lanes=lanes,
        endpoints=endpoints,
        entry=entry,
        target=target,
        planned=planned,
        span=span,
        moving=moving,
        min_clearance=min_clearance,
        column_count=len(columns),
        column_x={n: x_start + column_of[n] * pitch for n in moving},
        band_x0=band_x0,
        band_x1=band_x1,
        pitch=pitch,
        needed=needed,
    )


@dataclass
class _GridElement:
    """A crossing tile or a bare corner of a column grid, and what each of its lanes does there."""

    x: float
    y: float
    #: per lane: "flat", "vert_up", "vert_down", or "corner_<entry|target>_<down|up>"
    roles: dict[str, str]
    #: a tile whose vertical lane's corner is attached: the corner's distance (um)
    entry_corner: float | None = None
    target_corner: float | None = None
    name: str = ""
    component: Component | None = None
    #: where the instance is placed
    placement: tuple[float, float] = (0.0, 0.0)

    def port_xy(self, port: str) -> tuple[float, float]:
        assert self.component is not None
        center = self.component.ports[port].dcenter
        return (self.placement[0] + float(center[0]), self.placement[1] + float(center[1]))

    def ports_of(self, lane: str) -> tuple[str, str]:
        """The ports `lane` enters and leaves the element by."""
        role = self.roles[lane]
        enter, leave = {
            "flat": ("l", "r"),
            "vert_up": ("b", "t"),
            "vert_down": ("t", "b"),
            "corner_entry_down": ("w", "b"),
            "corner_entry_up": ("w", "t"),
            "corner_target_down": ("t", "e"),
            "corner_target_up": ("b", "e"),
        }[role]
        if role in ("vert_up", "vert_down"):
            if self.entry_corner is not None:
                enter = "w"
            if self.target_corner is not None:
                leave = "e"
        return enter, leave


class _BareCorner(NamedTuple):
    """A corner of a lane that is pre-wired as a bend of its own."""

    lane: str
    entry: bool
    down: bool
    x: float
    y: float


class ColumnGridPlacement(NamedTuple):
    """What `place_column_grid` placed for one band."""

    tile_names: list[str]
    builds: dict[str, CrossingGridBuildResult]
    #: original net -> its two stub nets
    stub_nets: dict[str, tuple[str, str]]
    #: original net -> the length it runs inside its tiles, in um
    lengths: dict[str, float]
    nets: list[Net]
    #: the original nets that were split
    split: set[str]
    expected_crossings: int
    placed_crossings: int


@dataclass
class _ColumnGridPlacer:
    """Places the tiles, corners and links of one planned column grid and splits its lanes."""

    plan: ColumnGridPlan
    geometry: CrossingGridGeometry
    #: the schematic the instances are added to
    derived: Schematic
    plus_tile: Component
    config: CrossingGridConfig
    #: the length of a lane inside one crossing (um)
    pass_length_um: float
    tile_names: list[str] = field(default_factory=list)
    #: per lane the elements it passes, in the order they were placed
    elements_of_lane: dict[str, list[_GridElement]] = field(default_factory=dict)
    link_count: int = 0
    nets: list[Net] = field(default_factory=list)
    stub_nets: dict[str, tuple[str, str]] = field(default_factory=dict)
    lengths: dict[str, float] = field(default_factory=dict)
    split: set[str] = field(default_factory=set)

    def planned_tiles(self) -> list[_GridElement]:
        """One tile per planned pair.

        With A left of B: at A's column on B's entry row when that row
        lies inside A's span, otherwise at B's column on A's target row.
        """
        plan, span = self.plan, self.plan.span
        infinity = float("inf")

        def x_of(n: str) -> float:
            return plan.column_x.get(n, infinity)

        tiles: list[_GridElement] = []
        for pair in sorted(plan.planned, key=lambda p: sorted(p)):
            # a left (or the moving one when b is level)
            a, b = sorted(sorted(pair), key=x_of)
            if x_of(a) == infinity:
                raise ValueError(f"column grid: planned crossing between two level lanes {a} x {b}")
            candidates = []
            if _strictly_inside(span[b][0], *span[a]):
                candidates.append((x_of(a), span[b][0], a))  # a vertical, b on its entry row
            if x_of(b) != infinity and _strictly_inside(span[a][1], *span[b]):
                candidates.append((x_of(b), span[a][1], b))  # b vertical, a on its target row
            if len(candidates) != 1:
                raise ValueError(
                    f"column grid: pair {a} x {b} has {len(candidates)} crossing points, expected 1"
                )
            x, y, vertical = candidates[0]
            direction = "vert_up" if span[vertical][1] > span[vertical][0] else "vert_down"
            # the pair's lanes by name: a set of names iterates in an order
            # that differs from process to process
            roles = {lane: (direction if lane == vertical else "flat") for lane in sorted(pair)}
            tiles.append(_GridElement(x=x, y=y, roles=roles))
        return tiles

    def bare_corners(self, tiles: list[_GridElement]) -> list[_BareCorner]:
        """Decide every corner of the moving lanes; returns the ones that get a bend of their own.

        A lane turns from its entry row into its column and from the
        column onto its target row. The router makes a corner only when
        both legs (to the nearest tile port, anchor or target) leave it
        corner_margin_um of straight room; otherwise the structure
        pre-wires it: as a crossing-with-bend tile when the nearest
        vertical tile is the tight one (marked on that tile), else as a
        bare bend.

        ``config.corners``: "always" (default): both corners of every
        moving lane are pre-wired, so the router only routes straight runs
        and no lane can stray across another lane's column
        (multiportmmi_32x32: a tile-less lane routed diagonally through a
        column). "auto": only corners whose legs are shorter than
        corner_margin_um. "router": none.
        """
        corners_mode = self.config.corners if self.config.corners is not None else "always"
        if corners_mode == "router":
            return []
        corners: list[_BareCorner] = []
        for lane in self.plan.moving:
            for is_entry in (True, False):
                corner = self._corner(lane, is_entry, tiles, always=corners_mode == "always")
                if corner is not None:
                    corners.append(corner)
        return corners

    def _corner(
        self, lane: str, is_entry: bool, tiles: list[_GridElement], *, always: bool
    ) -> _BareCorner | None:
        """One corner of one lane: None when the router turns it or a tile carries it."""
        plan = self.plan
        eps = _COLUMN_GRID_EPS
        corner_margin = float(self.geometry.corner_margin_um)
        half_tile = 0.5 * self.pass_length_um
        x_col = plan.column_x[lane]
        ye, yt = plan.span[lane]
        down = yt < ye
        corner_y = ye if is_entry else yt
        mine = [t for t in tiles if lane in t.roles]
        vertical_tiles = sorted(
            (t for t in mine if t.roles[lane] != "flat"), key=lambda t: t.y, reverse=down
        )
        first_vertical = (
            (vertical_tiles[0] if is_entry else vertical_tiles[-1]) if vertical_tiles else None
        )
        vertical_leg = (
            abs(first_vertical.y - corner_y) - half_tile if first_vertical else abs(yt - ye)
        )
        row_xs = [t.x for t in mine if t.roles[lane] == "flat" and abs(t.y - corner_y) < eps]
        if is_entry:
            left = [x for x in row_xs if x < x_col]
            horizontal_leg = (
                (x_col - max(left) - half_tile) if left else (x_col - plan.entry[lane][0])
            )
        else:
            right = [x for x in row_xs if x > x_col]
            horizontal_leg = (
                (min(right) - x_col - half_tile) if right else (plan.target[lane][0] - x_col)
            )
        if not always and vertical_leg >= corner_margin and horizontal_leg >= corner_margin:
            return None  # the router turns this one
        if first_vertical is None or abs(first_vertical.y - corner_y) >= corner_margin:
            return _BareCorner(lane=lane, entry=is_entry, down=down, x=x_col, y=corner_y)
        d = abs(first_vertical.y - corner_y)
        if d < CORNER_TILE_MIN_UM - eps:
            raise ValueError(
                f"column grid: lane {lane} crosses {d:.1f} um from its own corner; "
                f"a corner tile needs {CORNER_TILE_MIN_UM:.0f} um"
            )
        if is_entry:
            first_vertical.entry_corner = d
        else:
            first_vertical.target_corner = d
        return None

    def check_column_spacing(self, tiles: list[_GridElement]) -> None:
        """Raise `ValueError` when two crossings of one column are closer than a crossing is long.

        Two crossings on one straight need disjoint half-size windows:
        centers at least one crossing length apart.
        """
        by_x: dict[float, list[float]] = {}
        for t in tiles:
            by_x.setdefault(round(t.x, 6), []).append(t.y)
        for x, ys in by_x.items():
            ys.sort()
            for lo, hi in itertools.pairwise(ys):
                if hi - lo < self.pass_length_um + 0.5:
                    raise ValueError(
                        f"column grid: two crossings {hi - lo:.1f} um apart "
                        f"on the column at x={x:.1f}"
                    )

    def place_tiles(self, tiles: list[_GridElement]) -> int:
        """Add the tiles to the schematic; returns how many carry a pre-wired corner."""
        corner_tiles = 0
        for index, t in enumerate(tiles):
            component = self.plus_tile
            if t.entry_corner is not None or t.target_corner is not None:
                vertical = next(n for n, role in t.roles.items() if role != "flat")
                component = crossing_corner_tile_component(
                    self.geometry,
                    direction="down" if t.roles[vertical] == "vert_down" else "up",
                    entry_d_um=t.entry_corner,
                    target_d_um=t.target_corner,
                )
                _register_grid_cell(component.name, component, access_length_um=6.0)
                corner_tiles += 1
            t.name = f"{CROSSING_TILE_PREFIX}stage{self.plan.stage_depth}_{index}"
            t.component = component
            t.placement = (round(t.x, 3), round(t.y, 3))
            self.derived.add_instance(
                t.name,
                Instance(component=component.name),
                Placement(x=t.placement[0], y=t.placement[1]),
            )
            self.tile_names.append(t.name)
            for lane in t.roles:
                self.elements_of_lane.setdefault(lane, []).append(t)
        return corner_tiles

    def place_bare_corners(self, corners: list[_BareCorner]) -> None:
        """Add the bends of the corners no tile carries to the schematic."""
        radius = float(self.geometry.bend_radius_um)
        for index, c in enumerate(corners):
            component = plain_corner_component(
                self.geometry, entry=c.entry, direction="down" if c.down else "up"
            )
            _register_grid_cell(component.name, component, access_length_um=6.0)
            name = f"{CROSSING_LINK_PREFIX}corner_stage{self.plan.stage_depth}_{index}"
            if c.entry:
                corner_x, corner_y = round(c.x - radius, 3), round(c.y, 3)
            else:
                corner_x = round(c.x, 3)
                corner_y = round(c.y + (radius if c.down else -radius), 3)
            self.derived.add_instance(
                name, Instance(component=component.name), Placement(x=corner_x, y=corner_y)
            )
            self.tile_names.append(name)
            role = ("corner_entry_" if c.entry else "corner_target_") + ("down" if c.down else "up")
            self.elements_of_lane.setdefault(c.lane, []).append(
                _GridElement(
                    x=c.x,
                    y=c.y,
                    roles={c.lane: role},
                    name=name,
                    component=component,
                    placement=(float(corner_x), float(corner_y)),
                )
            )

    def _path_position(self, lane: str, element: _GridElement) -> tuple[float, float]:
        """Where `element` comes on the lane's way: entry row, entry corner, column, target corner, target row."""
        span = self.plan.span[lane]
        role = element.roles[lane]
        if role == "flat":
            on_entry = abs(element.y - span[0]) < _COLUMN_GRID_EPS
            return (0 if on_entry else 2, element.x)
        if role.startswith("corner_entry"):
            return (0.5, 0.0)
        if role.startswith("corner_target"):
            return (1.5, 0.0)
        return (1, element.y if span[1] > span[0] else -element.y)

    def _link(self, start: tuple[float, float], end: tuple[float, float]) -> None:
        """Pre-wire the straight from `start` to `end`: a run too short for a router net."""
        dx, dy = end[0] - start[0], end[1] - start[1]
        eps = _COLUMN_GRID_EPS
        straight = _straight(self.geometry, math.hypot(dx, dy))
        _register_plain_cell(straight.name, straight)
        rotation = (
            0 if abs(dy) < eps and dx > 0 else (180 if abs(dy) < eps else (90 if dy > 0 else 270))
        )
        link_name = f"{CROSSING_LINK_PREFIX}stage{self.plan.stage_depth}_{self.link_count}"
        self.link_count += 1
        self.derived.add_instance(
            link_name,
            Instance(component=straight.name),
            Placement(x=round(start[0], 3), y=round(start[1], 3), rotation=rotation),
        )
        self.tile_names.append(link_name)

    def split_lane(self, lane: str) -> None:
        """Replace one lane by the nets between its port, its elements in order, and its other port.

        Two elements closer than the router can route a net between
        (`MIN_ROUTER_GAP_UM`) on one line are joined by a pre-wired
        straight, or abut, and no net is made between them.
        """
        eps = _COLUMN_GRID_EPS
        elements = self.elements_of_lane[lane]
        elements.sort(key=lambda element: self._path_position(lane, element))
        p1, p2 = self.plan.endpoints[lane]
        previous = p1
        previous_xy: tuple[float, float] | None = None
        names: list[str] = []
        for element in elements:
            enter, leave = element.ports_of(lane)
            enter_xy = element.port_xy(enter)
            joined = False
            if previous_xy is not None:
                dx = enter_xy[0] - previous_xy[0]
                dy = enter_xy[1] - previous_xy[1]
                gap = math.hypot(dx, dy)
                joined = (abs(dx) < eps or abs(dy) < eps) and gap < MIN_ROUTER_GAP_UM - eps
                if joined and gap > eps:
                    self._link(previous_xy, enter_xy)
            if not joined:
                segment = (
                    f"{lane}{TO_GRID_SUFFIX}"
                    if not names
                    else f"{lane}{VIA_GRID_SUFFIX}{len(names)}"
                )
                self.nets.append(Net(p1=previous, p2=f"{element.name},{enter}", name=segment))
                names.append(segment)
            previous = f"{element.name},{leave}"
            previous_xy = element.port_xy(leave)
        final = f"{lane}{FROM_GRID_SUFFIX}"
        self.nets.append(Net(p1=previous, p2=p2, name=final))
        names.append(final)
        self.stub_nets[lane] = (names[0], names[-1])
        self.lengths[lane] = self.pass_length_um * len(elements)
        self.split.add(lane)


def place_column_grid(
    plan: ColumnGridPlan,
    *,
    geometry: CrossingGridGeometry,
    derived: Schematic,
    plus_tile: Component,
    config: CrossingGridConfig | None = None,
) -> ColumnGridPlacement:
    """Place a planned column grid in ``derived`` and split its lanes.

    The router connects the pieces: straight runs between tiles and, where
    ``config.corners`` leaves them to it, the corners of a lane. Raises
    `ValueError` when a planned pair has no single crossing point, a lane
    crosses too close to its own corner, or two crossings of one column
    are too close.
    """
    config = config if config is not None else CrossingGridConfig()
    depth = plan.stage_depth
    _LOGGER.info(
        f"      - column grid stage {depth}: {len(plan.lanes)} lanes, "
        f"{len(plan.moving)} moving, {plan.column_count} columns at {plan.pitch:.0f} um "
        f"({plan.needed:.0f} of {plan.band_x1 - plan.band_x0:.0f} um), "
        f"{len(plan.planned)} crossings"
    )
    if config.trace_column_grid:
        for n in plan.lanes:
            partners = sorted(m for m in plan.lanes if frozenset((n, m)) in plan.planned)
            _LOGGER.info(
                f"        lane {n}: entry ({plan.entry[n][0]:.1f}, {plan.entry[n][1]:.3f}) -> target "
                f"({plan.target[n][0]:.1f}, {plan.target[n][1]:.3f}) column x="
                f"{plan.column_x.get(n, float('nan')):.1f} crosses {partners}"
            )
    placer = _ColumnGridPlacer(
        plan=plan,
        geometry=geometry,
        derived=derived,
        plus_tile=plus_tile,
        config=config,
        pass_length_um=2.0 * _crossing_half_extent_um(_crossing_component()),
    )
    tiles = placer.planned_tiles()
    corners = placer.bare_corners(tiles)
    placer.check_column_spacing(tiles)
    corner_tiles = placer.place_tiles(tiles)
    placer.place_bare_corners(corners)
    if corner_tiles or corners:
        _LOGGER.info(
            f"      - column grid stage {depth}: {corner_tiles} tile(s) with a pre-wired corner, "
            f"{len(corners)} bare corner(s)"
        )
    for lane in list(placer.elements_of_lane):
        placer.split_lane(lane)
    builds = {
        f"stage{depth}": CrossingGridBuildResult(
            component=plus_tile,
            crossing_count=len(tiles),
            lane_length_um_by_net_name={},
            input_port_name_by_net_name={},
            output_port_name_by_net_name={},
            width_um=plan.needed,
            height_um=max(max(plan.span[n]) for n in plan.lanes)
            - min(min(plan.span[n]) for n in plan.lanes),
        )
    }
    return ColumnGridPlacement(
        tile_names=placer.tile_names,
        builds=builds,
        stub_nets=placer.stub_nets,
        lengths=placer.lengths,
        nets=placer.nets,
        split=placer.split,
        expected_crossings=len(plan.planned),
        placed_crossings=len(tiles),
    )


def derive_preplaced_crossing_layout(
    schematic: Schematic,
    unrouted_layout: Component,
    schedule: StageSchedule,
    *,
    geometry: CrossingGridGeometry | None = None,
    router_probe_options: SessionOptions | None = None,
    config: CrossingGridConfig | None = None,
) -> DerivedCrossingLayout:
    """Derive a schematic/layout pair with crossing grids placed and nets split.

    ``schedule`` is the placement's `CrossingDemand.stage_schedule()`: the
    bands with crossing events, left to right.

    ``router_probe_options`` (tiles mode, column grid): the routing run's
    options (obstacle config, fan-out access mode, bend radius), so the
    entry tiles can be placed on the router's static fan-out anchors.

    ``unrouted_layout`` (the benchmark's own unrouted layout) is only used to
    measure where each layer's free band is; the returned layout is rebuilt from
    the derived schematic with ``layout_from_schematic`` so it is an ordinary
    layout in every respect.
    """

    config = config if config is not None else CrossingGridConfig()
    geometry = geometry or crossing_grid_geometry_from_config(config)
    derived = Schematic()
    for instance_name, instance in schematic.netlist.instances.items():
        derived.add_instance(instance_name, instance, schematic.placements[instance_name])

    tiles_result = _derive_crossing_tiles(
        schematic,
        unrouted_layout,
        schedule,
        geometry,
        derived,
        router_probe_options,
        config=config,
    )
    (
        grid_instance_names,
        grid_builds,
        stub_nets,
        lengths,
        segment_nets,
        interstage_net_names,
        expected_crossings,
        placed_crossings,
        layer_decisions,
        fallback_net_names,
        dense_fanout_instances,
    ) = tiles_result
    for net_name in schematic.netlist.routes:
        if net_name in interstage_net_names:
            continue
        p1, p2 = net_endpoints(schematic, net_name)
        derived.add_net(Net(p1=p1, p2=p2, name=net_name))
    for net in segment_nets:
        derived.add_net(net)
    derived_layout = layout_from_schematic(derived)
    return DerivedCrossingLayout(
        schematic=derived,
        unrouted_layout=derived_layout,
        grid_instance_names=grid_instance_names,
        stub_nets_by_original_net=stub_nets,
        grid_internal_length_um_by_original_net=lengths,
        expected_crossing_count=expected_crossings,
        placed_crossing_count=placed_crossings,
        grid_builds=grid_builds,
        layer_decisions=layer_decisions,
        router_fallback_net_names=fallback_net_names,
        plan_event_count=schedule.event_count,
        dense_fanout_instances=dense_fanout_instances,
    )


def preplaced_crossing_grid_metrics(derived: DerivedCrossingLayout) -> dict[str, object]:
    """Compact, JSON-friendly summary for verification reports."""
    return {
        "grid_count": len(derived.grid_instance_names),
        "grid_instance_names": list(derived.grid_instance_names),
        "expected_crossing_count": int(derived.expected_crossing_count),
        "crossing_component_count": int(derived.placed_crossing_count),
        "split_net_count": len(derived.stub_nets_by_original_net),
        "grid_widths_um": {
            name: round(build.width_um, 3) for name, build in derived.grid_builds.items()
        },
        "grid_heights_um": {
            name: round(build.height_um, 3) for name, build in derived.grid_builds.items()
        },
        "layer_structures": [
            decision.describe() if hasattr(decision, "describe") else str(decision)
            for decision in derived.layer_decisions
        ],
        "router_fallback_net_count": len(derived.router_fallback_net_names),
        "plan_event_count": int(derived.plan_event_count),
    }
