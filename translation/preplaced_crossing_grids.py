"""Pre-placed, topology-derived crossing tiles for permutation-network benchmarks.

For a Benes-style network the set of waveguide crossings in every interstage
layer is fully determined by the graph before routing starts. This module turns
that precomputed information (a ``CrossingPlan``) into placed PDK crossing
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
Per layer, ``translation.crossing_structures.select_layer_structure`` picks the
X array (the swap levels are columns spread evenly over the layer's free band,
each crossing at the midpoint of the two rows it swaps) or the column grid
(multiport MMIs), or leaves the layer to the guided router. The pre-wired
lattice and stretched-array builders that preceded the tiles, and the
experimental line placement, were removed in phase 4 slice A of the 2026-10
do-over; no paper cell used them.
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, replace

import gdsfactory as gf
from gdsfactory.component import Component
from gdsfactory.schematic import Instance, Net, Placement, Schematic
from photonic_router.config import CrossingGridConfig, RoutingConfig
from photonic_router.crossing_plan import CrossingPlan, CrossingStagePlan
from photonic_router.routing_layers import (
    ComponentPortAccessRule,
    register_component_port_access_rule,
)

from translation.layout_from_schematic import layout_from_schematic
from translation.route_gds import get_port_from_instance

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
    component: Component
    crossing_count: int
    lane_length_um_by_net_name: dict[str, float]
    input_port_name_by_net_name: dict[str, str]
    output_port_name_by_net_name: dict[str, str]
    width_um: float
    height_um: float


@dataclass
class DerivedCrossingLayout:
    schematic: Schematic
    unrouted_layout: Component
    grid_instance_names: list[str]
    stub_nets_by_original_net: dict[str, tuple[str, str]]
    grid_internal_length_um_by_original_net: dict[str, float]
    expected_crossing_count: int
    placed_crossing_count: int
    grid_builds: dict[str, CrossingGridBuildResult] = field(default_factory=dict)
    # tiles mode: the selector's decision per crossing layer (see
    # translation/crossing_structures.py), in stage order
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


def _lane_movement(
    stage_plan: CrossingStagePlan,
) -> tuple[dict[str, tuple[int, int, int]], list[tuple[int, int, object, object]]]:
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
    crossings: list[tuple[int, int, object, object]] = []
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


def _net_endpoints(schematic: Schematic, net_name: str) -> tuple[str, str]:
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
            port_names=tuple(port.name for port in component.ports),
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
    by_corner: dict[str, object] = {}
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
    unrouted_layout: Component, router_probe_kwargs: Mapping[str, object] | None
) -> tuple[float, float, float] | None:
    """The routing grid's (origin_x, origin_y, cell size) the flow will use.

    Mirrors `photonic_router.static_obstacle_builder.make_grid_spec`: the
    layout's bounding box expanded by the obstacle config's security margin,
    unless the config pins a die bbox. None when no probe config is given
    (unit tests without a router configuration): tiles then stay unsnapped.
    """
    if not router_probe_kwargs:
        return None
    config = router_probe_kwargs.get("obstacle_config")
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


def _derive_crossing_tiles(
    schematic: Schematic,
    unrouted_layout: Component,
    crossing_plan: CrossingPlan,
    geometry: CrossingGridGeometry,
    derived: Schematic,
    router_probe_kwargs: Mapping[str, object] | None = None,
    config: CrossingGridConfig | None = None,
) -> tuple:
    """Place one crossing tile per planned crossing and split every crossing
    lane into router nets between its tiles (see ``crossing_tile_component``).

    Tile positions follow the stretched-array rule: the swap levels of a
    stage are columns spread evenly over the layer's free band minus
    ``band_margin_um`` on each side, and a crossing between the lanes in
    slots ``s`` and ``s + 1`` sits at the midpoint of those lanes' natural
    source rows -- for a single swap exactly the average of the four ports
    involved (owner rule, 2026-09-07).
    """
    config = config if config is not None else CrossingGridConfig()
    tile = crossing_tile_component(geometry)
    _register_grid_cell(tile.name, tile)
    anchors: dict[str, tuple[float, float]] = {}
    dense_fanout_instances: frozenset[str] | None = None
    from translation.crossing_structures import evaluate_x_array
    from translation.route_rust import static_fanout_anchors_um

    # Every source instance with two or more crossing lanes in a
    # column-grid layer gets static fan-out stubs (the router's default
    # threshold is three), so those lanes start on spread rows; the
    # router reads the same variable in the routing run. X-array layers
    # (Benes) keep the router's default: their tiles sit on the natural
    # rows and a stub would move the lane away from them.

    dense: set[str] = set()
    for stage_plan in crossing_plan.stages.values():
        if not stage_plan.events:
            continue
        # A layer the X array can take keeps the router's default stubs
        # (its tiles sit on the natural rows); every other crossing layer
        # is a column-grid candidate and needs its lanes on spread rows.
        if evaluate_x_array(schematic, unrouted_layout, stage_plan, geometry).feasible:
            continue
        count: dict[str, int] = {}
        for edge in stage_plan.initial_edge_order:
            count[edge.source.instance] = count.get(edge.source.instance, 0) + 1
        dense.update(name for name, n in count.items() if n >= 2)
    dense_fanout_instances = frozenset(dense) if dense else None
    probe_kwargs = dict(router_probe_kwargs or {})
    probe_config = probe_kwargs.get("config") or RoutingConfig()
    probe_kwargs["config"] = replace(
        probe_config,
        fanout=replace(probe_config.fanout, dense_fanout_instances=dense_fanout_instances),
    )
    anchors = static_fanout_anchors_um(unrouted_layout, schematic, **probe_kwargs)
    print(
        f"      - crossing tiles: {len(anchors)} static fan-out anchor(s) from the router "
        f"(dense instances: {', '.join(sorted(dense)) or 'none'})"
    )
    plus_tile = crossing_tile_component(geometry, axis_aligned=True)
    _register_grid_cell(plus_tile.name, plus_tile, access_length_um=6.0)
    crossing = _crossing_component()
    pass_length_um = 2.0 * _crossing_half_extent_um(crossing)
    tile_names: list[str] = []
    builds: dict[str, CrossingGridBuildResult] = {}
    stub_nets: dict[str, tuple[str, str]] = {}
    lengths: dict[str, float] = {}
    nets: list[Net] = []
    split: set[str] = set()
    expected = 0
    placed = 0
    decisions: list[object] = []
    fallback_layers: list[object] = []
    fallback_nets: set[str] = set()
    for stage_key in sorted(crossing_plan.stages):
        stage_plan = crossing_plan.stages[stage_key]
        if not stage_plan.initial_edge_order or not stage_plan.events:
            continue
        lanes = list(stage_plan.initial_edge_order)
        rows: list[float] = []
        source_xs: list[float] = []
        target_xs: list[float] = []
        target_rows: list[float] = []
        for edge in lanes:
            p1, p2 = _net_endpoints(schematic, edge.net_name)
            source = get_port_from_instance(unrouted_layout, *p1.split(","))
            target = get_port_from_instance(unrouted_layout, *p2.split(","))
            rows.append(float(source.dcenter[1]))
            source_xs.append(float(source.dcenter[0]))
            target_xs.append(float(target.dcenter[0]))
            target_rows.append(float(target.dcenter[1]))
        from translation.crossing_structures import (
            COLUMN_GRID,
            ROUTER,
            X_ARRAY,
            select_layer_structure,
        )

        allowed = (X_ARRAY, COLUMN_GRID)
        decision = select_layer_structure(
            schematic,
            unrouted_layout,
            stage_plan,
            geometry,
            anchors,
            stage_key=tuple(stage_key),
            allowed=allowed,
            config=config,
        )
        decisions.append(decision)
        print(f"      - crossing structure {decision.describe()}")
        if decision.chosen == ROUTER:
            fallback_layers.append(decision)
            fallback_nets.update(edge.net_name for edge in stage_plan.initial_edge_order)
            continue
        if decision.chosen == COLUMN_GRID:
            stage_result = _column_grid_stage(
                schematic,
                unrouted_layout,
                stage_plan,
                geometry,
                derived,
                plus_tile,
                anchors,
                config=config,
            )
            tile_names.extend(stage_result[0])
            builds.update(stage_result[1])
            stub_nets.update(stage_result[2])
            lengths.update(stage_result[3])
            nets.extend(stage_result[4])
            split.update(stage_result[5])
            expected += stage_result[6]
            placed += stage_result[7]
            continue
        for upper, lower in itertools.pairwise(rows):
            if upper <= lower:
                raise ValueError(
                    "crossing tiles need the stage's lanes ordered top to bottom by their "
                    f"source rows; stage {stage_key} rows {rows}"
                )
        movement, crossings = _lane_movement(stage_plan)
        level_count = max(level for level, *_ in crossings) + 1
        band_x0 = max(source_xs)
        band_x1 = min(target_xs)
        usable = (band_x1 - band_x0) - 2.0 * float(geometry.band_margin_um)
        pitch = usable / float(level_count)
        if pitch < tile.info.get("min_pitch_um", 0.0):
            raise ValueError("crossing tiles: band too narrow")
        tile_by_key: dict[tuple[int, int], str] = {}
        key_by_lane_level: dict[tuple[str, int], tuple[int, int]] = {}
        depth = stage_plan.source_depth
        grid_origin = _router_grid_origin(unrouted_layout, router_probe_kwargs)
        for level, upper_slot, upper, lower in crossings:
            name = f"{CROSSING_TILE_PREFIX}stage{depth}_l{level}_s{upper_slot}"
            x = band_x0 + float(geometry.band_margin_um) + (level + 0.5) * pitch
            y = 0.5 * (rows[upper_slot] + rows[upper_slot + 1])
            x, y = _snap_x_tile_center(x, y, grid_origin)
            derived.add_instance(
                name, Instance(component=tile.name), Placement(x=round(x, 3), y=round(y, 3))
            )
            tile_by_key[(level, upper_slot)] = name
            key_by_lane_level[(upper.net_name, level)] = (level, upper_slot)
            key_by_lane_level[(lower.net_name, level)] = (level, upper_slot)
            tile_names.append(name)
            placed += 1
        expected += len(stage_plan.events)
        builds[f"stage{depth}"] = CrossingGridBuildResult(
            component=tile,
            crossing_count=len(crossings),
            lane_length_um_by_net_name={},
            input_port_name_by_net_name={},
            output_port_name_by_net_name={},
            width_um=usable,
            height_um=max(rows) - min(rows),
        )
        for edge in lanes:
            net_name = edge.net_name
            if net_name not in movement:
                continue  # a lane that crosses nothing stays an ordinary net
            first, last, direction = movement[net_name]
            enter, leave = ("ul", "lr") if direction > 0 else ("ll", "ur")
            p1, p2 = _net_endpoints(schematic, net_name)
            previous = p1
            names: list[str] = []
            for index, level in enumerate(range(first, last + 1)):
                tile_name = tile_by_key[key_by_lane_level[(net_name, level)]]
                if index == 0:
                    segment = f"{net_name}{TO_GRID_SUFFIX}"
                else:
                    segment = f"{net_name}{VIA_GRID_SUFFIX}{index}"
                nets.append(Net(p1=previous, p2=f"{tile_name},{enter}", name=segment))
                names.append(segment)
                previous = f"{tile_name},{leave}"
            final = f"{net_name}{FROM_GRID_SUFFIX}"
            nets.append(Net(p1=previous, p2=p2, name=final))
            names.append(final)
            stub_nets[net_name] = (names[0], names[-1])
            lengths[net_name] = pass_length_um * (last - first + 1)
            split.add(net_name)
    if fallback_layers and not geometry.router_fallback:
        raise ValueError(
            "no pre-placed crossing structure fits: "
            + " | ".join(d.describe() for d in fallback_layers)  # type: ignore[attr-defined]
        )
    if fallback_layers:
        print(
            f"      - crossing structure: {len(fallback_layers)} layer(s) with "
            f"{len(fallback_nets)} net(s) left to the guided router"
        )
    return (
        tile_names,
        builds,
        stub_nets,
        lengths,
        nets,
        split,
        expected,
        placed,
        decisions,
        fallback_nets,
        dense_fanout_instances,
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

    def _corner(side: str, d_um: float, turn_to_west: bool) -> object:
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


def _column_grid_stage(
    schematic: Schematic,
    unrouted_layout: Component,
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry,
    derived: Schematic,
    plus_tile: Component,
    anchors: Mapping[str, tuple[float, float]],
    probe_only: bool = False,
    config: CrossingGridConfig | None = None,
) -> (
    tuple[
        list[str],
        dict[str, CrossingGridBuildResult],
        dict[str, tuple[str, str]],
        dict[str, float],
        list[Net],
        set[str],
        int,
        int,
    ]
    | dict[str, float | int]
):
    """Column grid for one stage: axis-aligned "+" crossings only.

    With ``probe_only`` nothing is placed; the return value is a summary
    dict (smallest corner clearance, columns, needed and available band,
    moving lanes, planned crossings, extra length over the octile minimum),
    and infeasible geometry still raises ``ValueError``.

    Every lane is horizontal (on its entry row: the router's static fan-out
    anchor if the port has one, else the port row), then vertical in its own
    column, then horizontal on its target row. Two lanes cross where one's
    vertical meets the other's horizontal. Which pairs cross depends only on
    the left-to-right order of the columns: a pair whose order flips crosses
    once either way; a pair that does not flip but whose vertical spans
    interleave crosses in one order and not in the other, which fixes the
    order. The order is built from those constraints (widest travel first as
    the tiebreak); lanes with disjoint spans and no constraint between them
    share a column. Tile of a planned pair with A left of B: at A's column on
    B's entry row when that row lies inside A's span, otherwise at B's column
    on A's target row. Lanes that do not move are pure horizontals (no
    column). The router connects the pieces: straight runs between tiles and
    two 90-degree corners per lane.
    """
    config = config if config is not None else CrossingGridConfig()
    crossing = _crossing_component()
    pass_length_um = 2.0 * _crossing_half_extent_um(crossing)
    eps = 1e-6
    lanes = [edge.net_name for edge in stage_plan.initial_edge_order]
    endpoints: dict[str, tuple[str, str]] = {}
    entry: dict[str, tuple[float, float]] = {}
    target: dict[str, tuple[float, float]] = {}
    for edge in stage_plan.initial_edge_order:
        p1, p2 = _net_endpoints(schematic, edge.net_name)
        endpoints[edge.net_name] = (p1, p2)
        if p1 in anchors:
            entry[edge.net_name] = anchors[p1]
        else:
            port = get_port_from_instance(unrouted_layout, *p1.split(","))
            entry[edge.net_name] = (float(port.dcenter[0]), float(port.dcenter[1]))
        if p2 in anchors:
            target[edge.net_name] = anchors[p2]
        else:
            port = get_port_from_instance(unrouted_layout, *p2.split(","))
            target[edge.net_name] = (float(port.dcenter[0]), float(port.dcenter[1]))
    planned = {frozenset((ev.edge_a.net_name, ev.edge_b.net_name)) for ev in stage_plan.events}
    span = {n: (entry[n][1], target[n][1]) for n in lanes}
    moving = [n for n in lanes if abs(span[n][0] - span[n][1]) > eps]

    def inside(v: float, a: float, b: float) -> bool:
        lo, hi = min(a, b), max(a, b)
        return lo + eps < v < hi - eps

    # A crossing on a lane's column must keep clear of that lane's own two
    # corners (entry row and target row): the crossing's straight window
    # plus a bend of radius 5 need about 12 um.
    corner_margin = float(geometry.corner_margin_um)

    def crossing_points_if_left(a: str, b: str) -> tuple[list[tuple[float, str]], float]:
        """Crossing rows with ``a`` left of ``b`` as ``(row, vertical lane)``,
        plus the smallest distance of those crossings to the vertical lane's
        own corners (inf when there is none)."""
        points: list[tuple[float, str]] = []
        clearance = float("inf")
        if inside(span[b][0], *span[a]):  # a's vertical x b's entry horizontal
            row = span[b][0]
            clearance = min(clearance, abs(row - span[a][0]), abs(row - span[a][1]))
            points.append((row, a))
        if inside(span[a][1], *span[b]):  # b's vertical x a's target horizontal
            row = span[a][1]
            clearance = min(clearance, abs(row - span[b][0]), abs(row - span[b][1]))
            points.append((row, b))
        return points, clearance

    before: dict[str, set[str]] = {n: set() for n in moving}
    tied: set[frozenset[str]] = set()
    min_clearance = float("inf")
    for a, b in itertools.combinations(moving, 2):
        need = 1 if frozenset((a, b)) in planned else 0
        pts_a, clear_a = crossing_points_if_left(a, b)
        pts_b, clear_b = crossing_points_if_left(b, a)
        count_a = len(pts_a) == need
        count_b = len(pts_b) == need
        # A corner tile (crossing with the bend attached) covers clearances
        # down to CORNER_TILE_MIN_UM, so ordering only needs that floor; the
        # router's larger corner_margin_um decides later which corners the
        # structure pre-wires.
        ok_a = count_a and clear_a >= CORNER_TILE_MIN_UM
        ok_b = count_b and clear_b >= CORNER_TILE_MIN_UM
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
        if ok_a:
            before[b].add(a)
            tied.add(frozenset((a, b)))
        else:
            before[a].add(b)
            tied.add(frozenset((a, b)))
    for n in lanes:
        if n in moving:
            continue
        for m in moving:
            need = 1 if frozenset((n, m)) in planned else 0
            has = 1 if inside(span[n][0], *span[m]) else 0
            if has != need:
                raise ValueError(
                    f"column grid: level lane {n} and {m} need {need} crossing(s), "
                    f"geometry gives {has}"
                )
    order: list[str] = []
    remaining = set(moving)
    while remaining:
        ready = [n for n in remaining if before[n] <= set(order)]
        if not ready:
            raise ValueError("column grid: the column order constraints are cyclic")
        ready.sort(key=lambda n: (-abs(span[n][0] - span[n][1]), n))
        order.append(ready[0])
        remaining.remove(ready[0])
    columns: list[list[str]] = []
    column_of: dict[str, int] = {}
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
            # Shared column: spans apart by more than a bare corner body plus
            # clearance (a corner bend of radius 5 plus the router's 2 um ring
            # on each side), so one lane's corner never sits on another
            # lane's vertical run.
            share_gap = float(geometry.bend_radius_um) * 2.0 + 6.0
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
    band_x0 = max(entry[n][0] for n in lanes) + float(geometry.column_lead_um)
    band_x1 = min(target[n][0] for n in lanes) - float(geometry.column_lead_um)
    pitch = float(geometry.column_pitch_um)
    needed = (len(columns) - 1) * pitch if columns else 0.0
    if needed > band_x1 - band_x0 + eps:
        raise ValueError(
            f"column grid: stage {stage_plan.source_depth} needs {len(columns)} columns "
            f"({needed:.1f} um) but the band leaves {band_x1 - band_x0:.1f} um"
        )
    x_start = 0.5 * (band_x0 + band_x1 - needed)
    column_x = {n: x_start + column_of[n] * pitch for n in moving}
    if probe_only:
        extra = sum(
            min(abs(span[n][0] - span[n][1]), band_x1 - band_x0) * (2.0 - math.sqrt(2.0))
            for n in moving
        )
        return {
            "min_clearance": min_clearance,
            "columns": len(columns),
            "needed_um": needed,
            "band_um": band_x1 - band_x0,
            "moving": len(moving),
            "lanes": len(lanes),
            "planned": len(planned),
            "extra_length_um": extra,
        }
    print(
        f"      - column grid stage {stage_plan.source_depth}: {len(lanes)} lanes, "
        f"{len(moving)} moving, {len(columns)} columns at {pitch:.0f} um "
        f"({needed:.0f} of {band_x1 - band_x0:.0f} um), {len(planned)} crossings"
    )
    if config.trace_column_grid:
        for n in lanes:
            partners = sorted(m for m in lanes if frozenset((n, m)) in planned)
            print(
                f"        lane {n}: entry ({entry[n][0]:.1f}, {entry[n][1]:.3f}) -> target "
                f"({target[n][0]:.1f}, {target[n][1]:.3f}) column x="
                f"{column_x.get(n, float('nan')):.1f} crosses {partners}"
            )
    infinity = float("inf")

    def x_of(n: str) -> float:
        return column_x.get(n, infinity)

    # tiles
    tiles: list[dict] = []
    for pair in sorted(planned, key=lambda p: sorted(p)):
        a, b = sorted(pair, key=x_of)  # a left (or the moving one when b is level)
        if x_of(a) == infinity:
            raise ValueError(f"column grid: planned crossing between two level lanes {a} x {b}")
        candidates = []
        if inside(span[b][0], *span[a]):
            candidates.append((x_of(a), span[b][0], a, b))  # a vertical, b on its entry row
        if x_of(b) != infinity and inside(span[a][1], *span[b]):
            candidates.append((x_of(b), span[a][1], b, a))  # b vertical, a on its target row
        if len(candidates) != 1:
            raise ValueError(
                f"column grid: pair {a} x {b} has {len(candidates)} crossing points, expected 1"
            )
        x, y, vertical, flat = candidates[0]
        direction = "vert_up" if span[vertical][1] > span[vertical][0] else "vert_down"
        tiles.append(
            {
                "x": x,
                "y": y,
                "roles": {vertical: direction, flat: "flat"},
                "pair": pair,
                "entry_corner": None,
                "target_corner": None,
            }
        )
    # Corners. A lane turns from its entry row into its column and from the
    # column onto its target row. The router makes a corner only when both
    # legs (to the nearest tile port, anchor or target) leave it
    # corner_margin_um of straight room; otherwise the structure pre-wires
    # it: as a crossing-with-bend tile when the nearest vertical tile is the
    # tight one, else as a bare bend.
    half_tile = 0.5 * pass_length_um
    corner_elements: list[dict] = []

    # "always" (default): both corners of every moving lane are pre-wired
    # (crossing-with-bend tile or bare bend), so the router only routes
    # straight runs and no lane can stray across another lane's column
    # (multiportmmi_32x32: a tile-less lane routed diagonally through a
    # column). "auto": only corners whose legs are shorter than
    # corner_margin_um. "router": none.
    corners_mode = config.corners if config.corners is not None else "always"
    for n in moving if corners_mode != "router" else []:
        x_col = column_x[n]
        ye, yt = span[n]
        down = yt < ye
        mine = [t for t in tiles if n in t["roles"]]
        vertical_tiles = sorted(
            (t for t in mine if t["roles"][n] != "flat"), key=lambda t: t["y"], reverse=down
        )
        entry_row_tiles = [t for t in mine if t["roles"][n] == "flat" and abs(t["y"] - ye) < eps]
        target_row_tiles = [t for t in mine if t["roles"][n] == "flat" and abs(t["y"] - yt) < eps]
        for is_entry in (True, False):
            corner_y = ye if is_entry else yt
            first_vertical = (
                (vertical_tiles[0] if is_entry else vertical_tiles[-1]) if vertical_tiles else None
            )
            vertical_leg = (
                abs(first_vertical["y"] - corner_y) - half_tile if first_vertical else abs(yt - ye)
            )
            if is_entry:
                left = [t["x"] for t in entry_row_tiles if t["x"] < x_col]
                horizontal_leg = (x_col - max(left) - half_tile) if left else (x_col - entry[n][0])
            else:
                right = [t["x"] for t in target_row_tiles if t["x"] > x_col]
                horizontal_leg = (
                    (min(right) - x_col - half_tile) if right else (target[n][0] - x_col)
                )
            if (
                corners_mode != "always"
                and vertical_leg >= corner_margin
                and horizontal_leg >= corner_margin
            ):
                continue  # the router turns this one
            if first_vertical is not None and abs(first_vertical["y"] - corner_y) < corner_margin:
                d = abs(first_vertical["y"] - corner_y)
                if d < CORNER_TILE_MIN_UM - eps:
                    raise ValueError(
                        f"column grid: lane {n} crosses {d:.1f} um from its own corner; "
                        f"a corner tile needs {CORNER_TILE_MIN_UM:.0f} um"
                    )
                first_vertical["entry_corner" if is_entry else "target_corner"] = d
            else:
                corner_elements.append(
                    {"lane": n, "entry": is_entry, "down": down, "x": x_col, "y": corner_y}
                )
    # spacing check on each column
    by_x: dict[float, list[float]] = {}
    for t in tiles:
        by_x.setdefault(round(t["x"], 6), []).append(t["y"])
    for x, ys in by_x.items():
        ys.sort()
        for lo, hi in itertools.pairwise(ys):
            # two crossings on one straight need disjoint half-size windows:
            # centers at least one crossing length apart
            if hi - lo < pass_length_um + 0.5:
                raise ValueError(
                    f"column grid: two crossings {hi - lo:.1f} um apart on the column at x={x:.1f}"
                )
    depth = stage_plan.source_depth
    tile_names: list[str] = []
    tiles_of_lane: dict[str, list[dict]] = {}
    corner_tiles = 0
    for index, t in enumerate(tiles):
        name = f"{CROSSING_TILE_PREFIX}stage{depth}_{index}"
        component = plus_tile
        if t["entry_corner"] is not None or t["target_corner"] is not None:
            vertical = next(n for n, role in t["roles"].items() if role != "flat")
            component = crossing_corner_tile_component(
                geometry,
                direction="down" if t["roles"][vertical] == "vert_down" else "up",
                entry_d_um=t["entry_corner"],
                target_d_um=t["target_corner"],
            )
            _register_grid_cell(component.name, component, access_length_um=6.0)
            corner_tiles += 1
        derived.add_instance(
            name,
            Instance(component=component.name),
            Placement(x=round(t["x"], 3), y=round(t["y"], 3)),
        )
        t["name"] = name
        t["component"] = component
        t["placement"] = (round(t["x"], 3), round(t["y"], 3))
        tile_names.append(name)
        for lane in t["pair"]:
            tiles_of_lane.setdefault(lane, []).append(t)
    for index, c in enumerate(corner_elements):
        component = plain_corner_component(
            geometry, entry=c["entry"], direction="down" if c["down"] else "up"
        )
        _register_grid_cell(component.name, component, access_length_um=6.0)
        name = f"{CROSSING_LINK_PREFIX}corner_stage{depth}_{index}"
        radius = float(geometry.bend_radius_um)
        if c["entry"]:
            placement = Placement(x=round(c["x"] - radius, 3), y=round(c["y"], 3))
        else:
            placement = Placement(
                x=round(c["x"], 3), y=round(c["y"] + (radius if c["down"] else -radius), 3)
            )
        derived.add_instance(name, Instance(component=component.name), placement)
        tile_names.append(name)
        role = ("corner_entry_" if c["entry"] else "corner_target_") + (
            "down" if c["down"] else "up"
        )
        element = {
            "x": c["x"],
            "y": c["y"],
            "roles": {c["lane"]: role},
            "pair": (c["lane"],),
            "name": name,
            "entry_corner": None,
            "target_corner": None,
            "corner": True,
            "component": component,
            "placement": (float(placement.x), float(placement.y)),
        }
        tiles_of_lane.setdefault(c["lane"], []).append(element)
    if corner_tiles or corner_elements:
        print(
            f"      - column grid stage {depth}: {corner_tiles} tile(s) with a pre-wired corner, "
            f"{len(corner_elements)} bare corner(s)"
        )
    link_index = 0

    def port_xy(element: dict, port: str) -> tuple[float, float]:
        center = element["component"].ports[port].dcenter
        return (
            element["placement"][0] + float(center[0]),
            element["placement"][1] + float(center[1]),
        )

    builds = {
        f"stage{depth}": CrossingGridBuildResult(
            component=plus_tile,
            crossing_count=len(tiles),
            lane_length_um_by_net_name={},
            input_port_name_by_net_name={},
            output_port_name_by_net_name={},
            width_um=needed,
            height_um=max(max(span[n]) for n in lanes) - min(min(span[n]) for n in lanes),
        )
    }
    nets: list[Net] = []
    stub_nets: dict[str, tuple[str, str]] = {}
    lengths: dict[str, float] = {}
    split: set[str] = set()
    for lane, lane_tiles in tiles_of_lane.items():
        up = span[lane][1] > span[lane][0]

        def path_key(t: dict, lane: str = lane, up: bool = up) -> tuple[float, float]:
            role = t["roles"][lane]
            if role == "flat":
                on_entry = abs(t["y"] - span[lane][0]) < eps
                return (0 if on_entry else 2, t["x"])
            if role.startswith("corner_entry"):
                return (0.5, 0.0)
            if role.startswith("corner_target"):
                return (1.5, 0.0)
            return (1, t["y"] if up else -t["y"])

        lane_tiles.sort(key=path_key)
        p1, p2 = endpoints[lane]
        previous = p1
        previous_xy: tuple[float, float] | None = None
        names: list[str] = []
        for t in lane_tiles:
            role = t["roles"][lane]
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
                if t["entry_corner"] is not None:
                    enter = "w"
                if t["target_corner"] is not None:
                    leave = "e"
            enter_xy = port_xy(t, enter)
            if previous_xy is not None:
                dx = enter_xy[0] - previous_xy[0]
                dy = enter_xy[1] - previous_xy[1]
                aligned = abs(dx) < eps or abs(dy) < eps
                gap = math.hypot(dx, dy)
                if aligned and gap < MIN_ROUTER_GAP_UM - eps:
                    # too short for a router net: pre-wire the straight
                    if gap > eps:
                        straight = _straight(geometry, gap)
                        _register_plain_cell(straight.name, straight)
                        rotation = (
                            0
                            if abs(dy) < eps and dx > 0
                            else (180 if abs(dy) < eps else (90 if dy > 0 else 270))
                        )
                        link_name = f"{CROSSING_LINK_PREFIX}stage{depth}_{link_index}"
                        link_index += 1
                        derived.add_instance(
                            link_name,
                            Instance(component=straight.name),
                            Placement(
                                x=round(previous_xy[0], 3),
                                y=round(previous_xy[1], 3),
                                rotation=rotation,
                            ),
                        )
                        tile_names.append(link_name)
                    previous = f"{t['name']},{leave}"
                    previous_xy = port_xy(t, leave)
                    continue
            if not names:
                segment = f"{lane}{TO_GRID_SUFFIX}"
            else:
                segment = f"{lane}{VIA_GRID_SUFFIX}{len(names)}"
            nets.append(Net(p1=previous, p2=f"{t['name']},{enter}", name=segment))
            names.append(segment)
            previous = f"{t['name']},{leave}"
            previous_xy = port_xy(t, leave)
        final = f"{lane}{FROM_GRID_SUFFIX}"
        nets.append(Net(p1=previous, p2=p2, name=final))
        names.append(final)
        stub_nets[lane] = (names[0], names[-1])
        lengths[lane] = pass_length_um * len(lane_tiles)
        split.add(lane)
    return tile_names, builds, stub_nets, lengths, nets, split, len(planned), len(tiles)


def derive_preplaced_crossing_layout(
    schematic: Schematic,
    unrouted_layout: Component,
    crossing_plan: CrossingPlan,
    *,
    geometry: CrossingGridGeometry | None = None,
    router_probe_kwargs: Mapping[str, object] | None = None,
    config: CrossingGridConfig | None = None,
) -> DerivedCrossingLayout:
    """Derive a schematic/layout pair with crossing grids placed and nets split.

    ``router_probe_kwargs`` (tiles mode, column grid): the routing run's
    configuration (obstacle config, fan-out access mode, bend radius), so the
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
        crossing_plan,
        geometry,
        derived,
        router_probe_kwargs,
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
    ) = tiles_result[:8]
    layer_decisions = list(tiles_result[8]) if len(tiles_result) > 8 else []
    fallback_net_names = set(tiles_result[9]) if len(tiles_result) > 9 else set()
    dense_fanout_instances = tiles_result[10] if len(tiles_result) > 10 else None
    for net_name in schematic.netlist.routes:
        if net_name in interstage_net_names:
            continue
        p1, p2 = _net_endpoints(schematic, net_name)
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
        plan_event_count=sum(len(stage.events) for stage in crossing_plan.stages.values()),
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


def build_crossing_plan_for_benchmark(
    schematic: Schematic,
    metadata: Mapping[str, object],
) -> CrossingPlan:
    """Build the router-facing crossing plan from benchmark topology metadata."""
    from photonic_router.crossing_plan import build_crossing_plan
    from photonic_router.topology_analysis import analyze_schematic_topology

    topology = analyze_schematic_topology(
        schematic,
        node_depths=metadata.get("node_depths") or None,  # type: ignore[arg-type]
        node_ranks=metadata.get("node_ranks") or None,  # type: ignore[arg-type]
        edge_ranks=metadata.get("edge_ranks") or None,  # type: ignore[arg-type]
    )
    return build_crossing_plan(topology)
