"""Pre-placed, topology-derived crossing grids for permutation-network benchmarks.

For a Benes-style network the set of waveguide crossings in every interstage
layer is fully determined by the graph before routing starts. This module turns
that precomputed information (a ``CrossingPlan``) into real, fully-wired
gdsfactory components -- one "crossing grid" per interstage layer -- and derives
a second schematic/layout pair in which those grids are placed as ordinary
instances and every interstage net is split into two crossing-free stubs:

    switch output port -> grid input port      (``<net>__to_grid``)
    grid output port   -> next switch input   (``<net>__from_grid``)

The routing stage then runs with crossings disabled, because every crossing
already lives inside a grid. The benchmark files themselves are never modified;
all derivation happens in memory inside the routing flow.

Grid geometry
-------------

A ``CrossingStagePlan`` describes each layer as a bubble-sort network: lanes
enter in ``initial_edge_order`` (top to bottom), a sequence of *adjacent swaps*
grouped into ``level``s is applied, and lanes leave in ``final_edge_order``.
The grid realizes one column per level. In a column, every swapping lane pair
is an X: each lane takes a 45-degree bend toward its partner, runs a diagonal,
and bends back; the two perpendicular diagonals meet at the column midpoint,
where a PDK crossing component (rotated to sit on the diagonals) is placed.
Non-swapping lanes cross the column as plain straights. All pieces are ordinary
PDK straights/bends/crossings connected port-to-port, so the geometry is exactly
what the router itself would emit for the same moves.
"""

from __future__ import annotations

import hashlib
import itertools
import math
from dataclasses import dataclass, field
from typing import Iterable, Mapping

import gdsfactory as gf
from gdsfactory.component import Component
from gdsfactory.schematic import Instance, Net, Placement, Schematic

from photonic_router.crossing_plan import CrossingPlan, CrossingStagePlan
from photonic_router.path_length_graph import RoutedEdgeKey
from photonic_router.routing_layers import (
    ComponentPortAccessRule,
    register_component_port_access_rule,
)
from translation.layout_from_schematic import layout_from_schematic
from translation.route_gds import get_port_from_instance

GRID_INSTANCE_PREFIX = "crossing_grid_stage"
TO_GRID_SUFFIX = "__to_grid"
FROM_GRID_SUFFIX = "__from_grid"
# Port-to-port `connect` snaps every piece to the 1 nm database unit, so a few
# nanometres of accumulated rounding across a column is expected and harmless.
_GEOMETRY_TOLERANCE_UM = 0.02
# Grid components are content-addressed by name (see `_grid_component_name`);
# the KLayout cell registry refuses duplicate names within one process, so a
# grid with an identical schedule and geometry is built once and reused.
_GRID_BUILD_CACHE: dict[str, "CrossingGridBuildResult"] = {}


@dataclass(frozen=True)
class CrossingGridGeometry:
    """Physical parameters of a crossing grid.

    ``lane_pitch_um`` is the vertical (and horizontal) spacing of the lattice
    core where the crossings live. ``entry_straight_um`` is a plain straight
    added behind every port so the ports have a clean axis-aligned approach.
    ``bend_radius_um`` is the Euler bend radius for every bend.
    ``fan_column_pitch_um`` is the x spacing between the vertical jogs that
    move lanes from their natural rows to their lattice slots (and back).
    ``port_pair_spread_um`` is how far sibling ports of one component (e.g. a
    switch's two outputs, 1.25 um apart) are pushed apart at the grid face so
    each stub lands in its own routing-grid cell. ``unrouted_sibling_clearance_um``
    is the minimum distance a grid port keeps from a sibling port that is *not*
    in the grid (a lane that crosses nothing and is routed as an ordinary net),
    so the grid's bounding box never covers that lane's natural row and it can
    run straight past the grid.
    """

    # 14 um (was 20): the stage-1/8 grid of benes_32x32 is 482 um wide at
    # 20 um and leaves 31 um of the 544.5 um interstage band on each side
    # for the 32-lane fan-in, which the router cannot route (2026-09-07);
    # at 14 um it is 392 um wide (76 um per side) and all four Benes sizes
    # route clean. Override: PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM.
    lane_pitch_um: float = 14.0
    bend_radius_um: float = 5.0
    entry_straight_um: float = 4.0
    fan_column_pitch_um: float = 4.0
    port_pair_spread_um: float = 2.0
    unrouted_sibling_clearance_um: float = 0.0
    # "router": the grid is only the compact crossing lattice (ports on the
    # lattice slot rows, short straight stubs) and the router bridges the
    # gap to the switch ports -- the repository owner's model: the grid
    # removes crossing complexity, not routing. "in_grid": the grid also
    # contains the vertical fan-in/fan-out to the lanes' natural rows.
    # "stretched" (2026-09-07, owner rule): no compact lattice at all -- the
    # slot rows are the lanes' natural rows (sibling ports of one switch
    # spread to +-slot_spread_um), the swap levels are columns spread evenly
    # over the layer's free band, every crossing sits at the midpoint of the
    # two rows it swaps, and a lane runs 45-degree arms through its
    # crossings joined by verticals/horizontals -- the crossings are
    # distributed over the layer where the shortest paths cross.
    # "tiles" (2026-09-07 afternoon, owner correction of "stretched"): the
    # pre-placed structure is ONLY the crossings -- one small crossing tile
    # per planned crossing at the stretched array's position (level column,
    # midpoint of the two swapped rows), and every connection (switch port
    # to tile, tile to tile, tile to switch port) is an ordinary net for the
    # router. No pre-wired lanes, no slot rows, no port jogs. Default.
    # PHOTONIC_ROUTER_CROSSING_GRID_FAN_MODE=stretched|router restores the
    # pre-wired array / the compact lattice for comparison.
    fan_mode: str = "tiles"
    # tiles mode: where a tile goes. "lines": at the intersection of the two
    # crossing lanes' straight source-to-target lines (X tile for lanes
    # moving in opposite directions, axis-aligned "+" tile when they move
    # the same way or one is level), tiles closer than tile_min_spacing_um
    # pushed apart; the order of a lane's tiles is their order along its
    # line. "columns": the stretched-array rule (level columns over the
    # band, midpoint of the two swapped source rows) -- the default; it
    # routes every Benes size. "lines" is experimental (2026-09-07): the
    # router needs room for two bends between consecutive 45-degree tiles
    # of one lane unless they lie on an exact diagonal, and the intersection
    # points crowd at dense MMI ports, so it fails on benes_8x8 and
    # multiportmmi_8x8 as it stands.
    tile_placement: str = "auto"
    tile_min_spacing_um: float = 14.0
    # column grid (multiportmmi): x spacing of the vertical lane columns
    # (8 um crossing cell + clearance) and the free run from the entry
    # anchors to the first column / from the last column to the targets
    # (room for one 90-degree bend of radius 5 plus a straight).
    # 16 um: with 12 um two adjacent columns crossing one horizontal leave a
    # 4 um (two-cell) straight between the tiles, below what the router can
    # route as a net.
    column_pitch_um: float = 16.0
    column_lead_um: float = 14.0
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
    # stretched mode: half-spread of a switch's two sibling slots (the two
    # rows are 2 * slot_spread_um apart, wide enough for a crossing footprint
    # between them and a router stub from the 1.25 um port pair).
    slot_spread_um: float = 11.0
    # stretched mode: free band kept at both edges of the layer for the
    # router's port-to-slot stubs (a 10 um jog with 5 um bends plus the
    # port lanes; 30 um is too tight for the router, 40 um routes); the
    # level columns share the rest.
    band_margin_um: float = 50.0
    # Extra stub length per lane rank from the grid's edge (middle lanes get
    # the longest stubs, like the multiport-MMI static stubs), so the router
    # starts each lane's route at a different x. 0 disables the stagger.
    stub_stagger_um: float = 6.0
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


# ---------------------------------------------------------------------------
# Grid component construction
# ---------------------------------------------------------------------------


def _bend_45(geometry: CrossingGridGeometry, angle: float) -> Component:
    return gf.components.bend_euler(
        angle=angle,
        radius=geometry.bend_radius_um,
        width=geometry.route_width_um,
        cross_section=geometry.cross_section,
    )


def _straight(geometry: CrossingGridGeometry, length_um: float) -> Component:
    return gf.components.straight(
        length=length_um,
        width=geometry.route_width_um,
        cross_section=geometry.cross_section,
    )


def _crossing_component() -> Component:
    return gf.components.crossing()


def _port_dx_dy(component: Component) -> tuple[float, float]:
    p1 = component.ports["o1"].dcenter
    p2 = component.ports["o2"].dcenter
    return float(p2[0] - p1[0]), float(p2[1] - p1[1])


def _crossing_half_extent_um(crossing: Component) -> float:
    """Distance from the crossing's center to one of its ports."""
    center = crossing.dbbox().center()
    port = crossing.ports["o1"].dcenter
    return math.hypot(float(port[0]) - float(center.x), float(port[1]) - float(center.y))


def _component_length_um(component: Component, fallback_um: float) -> float:
    raw = component.info.get("length") if hasattr(component, "info") else None
    try:
        return float(raw) if raw is not None else float(fallback_um)
    except (TypeError, ValueError):
        return float(fallback_um)


def _grid_component_name(
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry,
    entry_rows: Mapping[str, float],
    exit_rows: Mapping[str, float],
) -> str:
    digest = hashlib.sha1(
        (
            repr([edge.net_name for edge in stage_plan.initial_edge_order])
            + repr([edge.net_name for edge in stage_plan.final_edge_order])
            + repr([(e.edge_a.net_name, e.edge_b.net_name, e.level) for e in stage_plan.events])
            + repr(geometry)
            + repr(sorted((k, round(v, 3)) for k, v in entry_rows.items()))
            + repr(sorted((k, round(v, 3)) for k, v in exit_rows.items()))
        ).encode("utf-8")
    ).hexdigest()[:10]
    return f"crossing_grid_s{stage_plan.source_depth}_{stage_plan.target_depth}_{digest}"


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


def _bend_s(geometry: CrossingGridGeometry, dx_um: float, dy_um: float) -> Component:
    return gf.components.bend_s(
        size=(dx_um, dy_um),
        width=geometry.route_width_um,
        cross_section=geometry.cross_section,
    )


def _fan_column_order(
    far_rows_by_net: Mapping[str, float],
    slot_rows_by_net: Mapping[str, float],
    *,
    closest_first: bool,
) -> dict[str, int]:
    """Assign each moving lane a fan column index (0 = leftmost).

    ``far_rows_by_net`` is the row a lane occupies on the far side of the fan
    (its natural entry or exit row) and ``slot_rows_by_net`` its lattice slot
    row. Lanes whose far row lies above their slot and lanes whose far row
    lies below never share a row range, so the two groups use the same column
    indices independently.

    Within a group the order must follow the far rows, not the travel
    distance: on the entry side the lane whose row is closest to the core
    turns first (leftmost), so an outer lane's later vertical run only passes
    rows its inner neighbours have already vacated; on the exit side the
    farthest lane turns first, for the mirrored reason. Ordering by travel
    distance is wrong because an outer lane can travel less than an inner one
    when its slot is also farther out.
    """
    order: dict[str, int] = {}
    for above in (True, False):
        group = [
            net
            for net, far_row in far_rows_by_net.items()
            if abs(far_row - slot_rows_by_net[net]) > _GEOMETRY_TOLERANCE_UM
            and (far_row > slot_rows_by_net[net]) == above
        ]
        # Distance from the core along the travel direction: for the group
        # above the core a lower far row is closer; below, a higher one is.
        group.sort(
            key=lambda net: far_rows_by_net[net] if above else -far_rows_by_net[net],
            reverse=not closest_first,
        )
        for index, net in enumerate(group):
            order[net] = index
    return order


def build_crossing_grid_component(
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry | None = None,
    *,
    entry_row_by_net: Mapping[str, float] | None = None,
    exit_row_by_net: Mapping[str, float] | None = None,
    band_width_um: float | None = None,
) -> CrossingGridBuildResult:
    """Build one fully-wired crossing grid for a single interstage layer.

    Every lane of ``stage_plan`` must take part in at least one crossing (see
    ``derive_preplaced_crossing_layout`` for how non-crossing lanes bypass the
    grid). The component has three horizontal regions:

    1. an entry fan-in: each lane enters at its natural row
       (``entry_row_by_net``, relative to the component's y origin; defaults
       to the lattice slot row) and moves vertically to its lattice slot with a
       90-degree jog in its own x column (a gentle S-bend when the distance is
       too small for two 90-degree bends);
    2. the lattice core: crossing centers sit on a diamond grid with
       horizontal and vertical spacing ``lane_pitch_um``, so a lane that swaps
       at consecutive levels runs one straight 45-degree diagonal through all
       of its crossings and 45-degree bends only occur where it starts and
       stops moving; waiting lanes run horizontally at their slot row;
    3. an exit fan-out mirroring the entry, to ``exit_row_by_net``.

    Ports: ``in_<i>`` on the left edge (orientation 180) for lane ``i`` of
    ``stage_plan.initial_edge_order`` (top to bottom), ``out_<j>`` on the right
    edge (orientation 0) for lane ``j`` of ``stage_plan.final_edge_order``.
    The lattice core is centered on the origin.
    """

    geometry = geometry or CrossingGridGeometry()
    stage_plan.validate()
    if geometry.fan_mode == "stretched":
        if band_width_um is None or entry_row_by_net is None:
            raise ValueError("stretched crossing grids need band_width_um and entry_row_by_net")
        return _build_stretched_grid_component(
            stage_plan,
            geometry,
            entry_row_by_net=entry_row_by_net,
            exit_row_by_net=exit_row_by_net or {},
            band_width_um=float(band_width_um),
        )
    lanes = list(stage_plan.initial_edge_order)
    lane_count = len(lanes)
    if lane_count == 0:
        raise ValueError("crossing grid needs at least one lane")
    pitch = float(geometry.lane_pitch_um)

    def slot_y(slot: int) -> float:
        return (0.5 * (lane_count - 1) - slot) * pitch

    final_slot_by_net = {edge.net_name: i for i, edge in enumerate(stage_plan.final_edge_order)}
    entry_rows = {
        edge.net_name: float(
            (entry_row_by_net or {}).get(edge.net_name, slot_y(index))
        )
        for index, edge in enumerate(lanes)
    }
    exit_rows = {
        edge.net_name: float(
            (exit_row_by_net or {}).get(edge.net_name, slot_y(final_slot_by_net[edge.net_name]))
        )
        for edge in lanes
    }
    component_name = _grid_component_name(stage_plan, geometry, entry_rows, exit_rows)
    cached = _GRID_BUILD_CACHE.get(component_name)
    if cached is not None:
        return cached
    movement, crossings = _lane_movement(stage_plan)
    idle = [edge.net_name for edge in lanes if edge.net_name not in movement]
    if idle:
        raise ValueError(
            "crossing grid lanes must all take part in a crossing; idle lanes: "
            + ", ".join(idle)
        )

    # --- lattice core parameters -------------------------------------------
    bend_down = _bend_45(geometry, -45.0)
    bend_up = _bend_45(geometry, 45.0)
    bend_dx, bend_dy = _port_dx_dy(bend_up)
    bend_dy = abs(bend_dy)
    bend_length_um = _component_length_um(bend_up, math.hypot(bend_dx, bend_dy))
    crossing = _crossing_component()
    crossing_half_um = _crossing_half_extent_um(crossing)
    crossing_pass_length_um = 2.0 * crossing_half_um
    arm_drop_um = 0.5 * pitch - bend_dy
    entry_arm_um = arm_drop_um * math.sqrt(2.0) - crossing_half_um
    if entry_arm_um <= 0.0:
        raise ValueError(
            f"lane_pitch_um={pitch} leaves no straight room between a 45-degree bend "
            f"(radius {geometry.bend_radius_um}) and an {crossing_pass_length_um:.1f} um "
            "crossing; increase lane_pitch_um"
        )
    link_um = pitch * math.sqrt(2.0) - crossing_pass_length_um
    if link_um <= 0.0:
        raise ValueError(f"lane_pitch_um={pitch} is smaller than the crossing itself")
    bend_overhang_um = bend_dx - bend_dy
    level_count = max(level for level, *_ in crossings) + 1
    core_margin_um = bend_overhang_um + 1.0
    core_width_um = 2.0 * core_margin_um + level_count * pitch
    core_x0 = -0.5 * core_width_um  # left edge of the core region

    # --- fan-in / fan-out parameters ----------------------------------------
    jog_down = _bend_90(geometry, -90.0)
    jog_up = _bend_90(geometry, 90.0)
    jog_dx, jog_dy = _port_dx_dy(jog_up)
    jog_dy = abs(jog_dy)
    jog_length_um = _component_length_um(jog_up, 0.5 * math.pi * geometry.bend_radius_um)
    jog_width_um = 2.0 * jog_dx  # x extent of a bend-vertical-bend jog
    column_pitch_um = float(geometry.fan_column_pitch_um)
    entry_um = float(geometry.entry_straight_um)

    entry_offsets = {net: slot_y(i) - entry_rows[net] for i, net in
                     ((i, e.net_name) for i, e in enumerate(lanes))}
    exit_offsets = {net: exit_rows[net] - slot_y(final_slot_by_net[net]) for net in entry_rows}
    entry_slot_rows = {e.net_name: slot_y(i) for i, e in enumerate(lanes)}
    exit_slot_rows = {net: slot_y(final_slot_by_net[net]) for net in entry_rows}
    entry_columns = _fan_column_order(entry_rows, entry_slot_rows, closest_first=True)
    exit_columns = _fan_column_order(exit_rows, exit_slot_rows, closest_first=False)
    entry_column_count = (max(entry_columns.values()) + 1) if entry_columns else 0
    exit_column_count = (max(exit_columns.values()) + 1) if exit_columns else 0

    def fan_width(column_count: int) -> float:
        if column_count == 0:
            return 0.0
        return (column_count - 1) * column_pitch_um + jog_width_um + 1.0

    entry_fan_um = fan_width(entry_column_count)
    exit_fan_um = fan_width(exit_column_count)

    def stub_len(slot: int) -> float:
        """Straight stub length for a lane at ``slot``: middle lanes longest."""
        rank_from_edge = min(slot, lane_count - 1 - slot)
        return entry_um + float(geometry.stub_stagger_um) * rank_from_edge

    max_stub_um = max(stub_len(slot) for slot in range(lane_count))
    x_left = core_x0 - entry_fan_um - max_stub_um
    x_right = core_x0 + core_width_um + exit_fan_um + max_stub_um
    total_width_um = x_right - x_left

    def crossing_center(level: int, upper_slot: int) -> tuple[float, float]:
        return (
            core_x0 + core_margin_um + (level + 0.5) * pitch,
            slot_y(upper_slot) - 0.5 * pitch,
        )

    component = Component(name=component_name)
    lane_length_um: dict[str, float] = {edge.net_name: 0.0 for edge in lanes}

    crossing_ref_by_key: dict[tuple[int, int], object] = {}
    for level, upper_slot, _upper, _lower in crossings:
        ref = component.add_ref(crossing)
        ref.rotate(-45.0)
        cx, cy = crossing_center(level, upper_slot)
        center = ref.dbbox().center()
        ref.dmove((cx - float(center.x), cy - float(center.y)))
        crossing_ref_by_key[(level, upper_slot)] = ref
    crossing_key_by_lane_level: dict[tuple[str, int], tuple[int, int]] = {}
    for level, upper_slot, upper, lower in crossings:
        crossing_key_by_lane_level[(upper.net_name, level)] = (level, upper_slot)
        crossing_key_by_lane_level[(lower.net_name, level)] = (level, upper_slot)

    def _check(port_a, port_b, what: str) -> None:
        a, b = port_a.dcenter, port_b.dcenter
        if math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])) > _GEOMETRY_TOLERANCE_UM:
            raise RuntimeError(
                f"crossing grid geometry mismatch ({what}): {tuple(a)} vs {tuple(b)}"
            )

    def _straight_to_x(open_port, x_target: float, net: str, what: str):
        """Append a horizontal straight from ``open_port`` to ``x_target``."""
        run_um = x_target - float(open_port.dcenter[0])
        if run_um < -_GEOMETRY_TOLERANCE_UM:
            raise RuntimeError(f"negative {what} straight for lane {net}: {run_um:.3f}")
        if run_um <= _GEOMETRY_TOLERANCE_UM:
            return open_port, 0.0
        ref = component.add_ref(_straight(geometry, run_um))
        ref.connect("o1", open_port)
        return ref.ports["o2"], run_um

    def _jog(open_port, dy_um: float, net: str):
        """Move vertically by ``dy_um`` (signed) starting at ``open_port``."""
        travel = abs(dy_um)
        if travel <= _GEOMETRY_TOLERANCE_UM:
            return open_port, 0.0
        if travel >= 2.0 * jog_dy + 1.0:
            first, second = (jog_up, jog_down) if dy_um > 0 else (jog_down, jog_up)
            ref = component.add_ref(first)
            ref.connect("o1", open_port)
            vertical_um = travel - 2.0 * jog_dy
            mid = component.add_ref(_straight(geometry, vertical_um))
            mid.connect("o1", ref.ports["o2"])
            ref2 = component.add_ref(second)
            ref2.connect("o1", mid.ports["o2"])
            return ref2.ports["o2"], 2.0 * jog_length_um + vertical_um
        ref = component.add_ref(_bend_s(geometry, jog_width_um, dy_um))
        ref.connect("o1", open_port)
        return ref.ports["o2"], _component_length_um(ref.cell, math.hypot(jog_width_um, dy_um))

    output_port_by_net: dict[str, str] = {}
    for start_slot, edge in enumerate(lanes):
        net = edge.net_name
        first, last, direction = movement[net]
        length = 0.0

        # Entry stub + port at the lane's natural row. The port sits at
        # x_left + (max_stub - own stub): shorter stubs start further right.
        entry_stub_um = stub_len(start_slot)
        ref = component.add_ref(_straight(geometry, entry_stub_um))
        ref.dmove((x_left + max_stub_um - entry_stub_um, entry_rows[net]))
        component.add_port(f"in_{start_slot}", port=ref.ports["o1"])
        open_port = ref.ports["o2"]
        length += entry_stub_um

        # Fan-in: turn in this lane's own column, land on the slot row.
        column = entry_columns.get(net)
        if column is not None:
            open_port, run = _straight_to_x(
                open_port, x_left + max_stub_um + column * column_pitch_um, net, "entry fan"
            )
            length += run
            open_port, run = _jog(open_port, entry_offsets[net], net)
            length += run
        if abs(float(open_port.dcenter[1]) - slot_y(start_slot)) > _GEOMETRY_TOLERANCE_UM:
            raise RuntimeError(f"lane {net} did not reach its entry slot row")

        # Wait until the first movement column, then bend toward the partner.
        bend_start_x = core_x0 + core_margin_um + first * pitch - bend_overhang_um
        open_port, run = _straight_to_x(open_port, bend_start_x, net, "wait")
        length += run
        ref = component.add_ref(bend_down if direction > 0 else bend_up)
        ref.connect("o1", open_port)
        open_port = ref.ports["o2"]
        length += bend_length_um

        enter_port, exit_port = ("o1", "o3") if direction > 0 else ("o4", "o2")
        for level in range(first, last + 1):
            arm_um = entry_arm_um if level == first else link_um
            ref = component.add_ref(_straight(geometry, arm_um))
            ref.connect("o1", open_port)
            x_ref = crossing_ref_by_key[crossing_key_by_lane_level[(net, level)]]
            _check(ref.ports["o2"], x_ref.ports[enter_port], f"lane {net} level {level}")
            open_port = x_ref.ports[exit_port]
            length += arm_um + crossing_pass_length_um
        ref = component.add_ref(_straight(geometry, entry_arm_um))
        ref.connect("o1", open_port)
        ref2 = component.add_ref(bend_up if direction > 0 else bend_down)
        ref2.connect("o1", ref.ports["o2"])
        open_port = ref2.ports["o2"]
        length += entry_arm_um + bend_length_um

        end_slot = final_slot_by_net[net]
        if abs(float(open_port.dcenter[1]) - slot_y(end_slot)) > _GEOMETRY_TOLERANCE_UM:
            raise RuntimeError(
                f"lane {net} ends at y={float(open_port.dcenter[1]):.4f}, "
                f"expected slot {end_slot} at {slot_y(end_slot):.4f}"
            )

        # Wait to the end of the core, then fan out to the natural exit row.
        open_port, run = _straight_to_x(open_port, core_x0 + core_width_um, net, "core exit")
        length += run
        column = exit_columns.get(net)
        if column is not None:
            open_port, run = _straight_to_x(
                open_port, core_x0 + core_width_um + column * column_pitch_um, net, "exit fan"
            )
            length += run
            open_port, run = _jog(open_port, exit_offsets[net], net)
            length += run
        if abs(float(open_port.dcenter[1]) - exit_rows[net]) > _GEOMETRY_TOLERANCE_UM:
            raise RuntimeError(f"lane {net} did not reach its exit row")
        exit_stub_um = stub_len(end_slot)
        open_port, run = _straight_to_x(open_port, x_right - max_stub_um, net, "exit")
        length += run
        # Place the exit stub absolutely (like the entry stub) rather than
        # chaining it: port-to-port connects accumulate ~1 nm of dbu
        # rounding, and the exit port must sit on *exactly* the row the next
        # switch port has, or the stub's endpoint corrector falls back from a
        # plain straight shift to a 12 um bump.
        ref = component.add_ref(_straight(geometry, exit_stub_um))
        ref.dmove((x_right - max_stub_um, exit_rows[net]))
        _check(ref.ports["o1"], open_port, f"lane {net} exit stub")
        component.add_port(f"out_{end_slot}", port=ref.ports["o2"])
        output_port_by_net[net] = f"out_{end_slot}"
        lane_length_um[net] = length + exit_stub_um

    input_port_by_net = {edge.net_name: f"in_{index}" for index, edge in enumerate(lanes)}
    component.info["crossing_count"] = len(crossings)
    component.info["lane_count"] = lane_count
    component.info["level_count"] = level_count
    ys = list(entry_rows.values()) + list(exit_rows.values()) + [slot_y(0), slot_y(lane_count - 1)]
    result = CrossingGridBuildResult(
        component=component,
        crossing_count=len(crossings),
        lane_length_um_by_net_name=lane_length_um,
        input_port_name_by_net_name=input_port_by_net,
        output_port_name_by_net_name=output_port_by_net,
        width_um=total_width_um,
        height_um=(max(ys) - min(ys)) + geometry.route_width_um,
    )
    _GRID_BUILD_CACHE[component_name] = result
    return result


def _rotate(vec: tuple[float, float], heading_deg: float) -> tuple[float, float]:
    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    return (vec[0] * c - vec[1] * s, vec[0] * s + vec[1] * c)


def _build_stretched_grid_component(
    stage_plan: CrossingStagePlan,
    geometry: CrossingGridGeometry,
    *,
    entry_row_by_net: Mapping[str, float],
    exit_row_by_net: Mapping[str, float],
    band_width_um: float,
) -> CrossingGridBuildResult:
    """Stretched crossing array (owner rule, 2026-09-07).

    Slot ``i`` of the lattice is the natural row of lane ``i`` of
    ``initial_edge_order`` (``entry_row_by_net``, relative to the component's
    y origin, strictly decreasing top to bottom). The swap levels are
    columns spread evenly over ``band_width_um - 2 * band_margin_um``; the
    crossing of level ``k`` between slots ``s`` and ``s + 1`` sits at
    ``(x_k, (row_s + row_s+1) / 2)``. A lane is: entry straight on its row,
    then for its movement block one 45-degree arm into each crossing and one
    out of it, joined between consecutive levels by a straight diagonal when
    the column pitch equals the row step, otherwise by a horizontal (pitch
    wider than the step) or a vertical (pitch narrower) run between two
    45-degree bends; after the last crossing it bends back onto its final
    slot row and runs to the exit straight. Ports: ``in_<i>`` on the left
    edge, ``out_<j>`` on the right edge, as in the compact lattice. The
    component is centered on x = 0 (the band center) and its ports lie on
    the slot rows, so the router only bridges ``band_margin_um`` and the
    row offset between a switch port and its slot.
    """

    lanes = list(stage_plan.initial_edge_order)
    lane_count = len(lanes)
    if lane_count == 0:
        raise ValueError("crossing grid needs at least one lane")
    rows = [float(entry_row_by_net[edge.net_name]) for edge in lanes]
    for upper, lower in itertools.pairwise(rows):
        if upper - lower < 2.0 * float(geometry.slot_spread_um) - _GEOMETRY_TOLERANCE_UM:
            raise ValueError(
                "stretched crossing grid needs slot rows strictly decreasing top to bottom "
                f"with at least {2.0 * geometry.slot_spread_um:.1f} um between them; got {rows}"
            )
    final_slot_by_net = {edge.net_name: i for i, edge in enumerate(stage_plan.final_edge_order)}
    entry_rows = {edge.net_name: rows[i] for i, edge in enumerate(lanes)}
    exit_rows = {edge.net_name: rows[final_slot_by_net[edge.net_name]] for edge in lanes}
    component_name = _grid_component_name(
        stage_plan, geometry, entry_rows, {**exit_rows, "__band__": round(band_width_um, 3)}
    )
    cached = _GRID_BUILD_CACHE.get(component_name)
    if cached is not None:
        return cached
    movement, crossings = _lane_movement(stage_plan)
    idle = [edge.net_name for edge in lanes if edge.net_name not in movement]
    if idle:
        raise ValueError(
            "crossing grid lanes must all take part in a crossing; idle lanes: " + ", ".join(idle)
        )

    # --- geometry pieces ----------------------------------------------------
    bend_ccw = _bend_45(geometry, 45.0)   # turns +45 degrees (counter-clockwise)
    bend_cw = _bend_45(geometry, -45.0)   # turns -45 degrees (clockwise)
    native_dx, native_dy = _port_dx_dy(bend_ccw)
    native_dy = abs(native_dy)
    bend_length_um = _component_length_um(bend_ccw, math.hypot(native_dx, native_dy))
    crossing = _crossing_component()
    crossing_half_um = _crossing_half_extent_um(crossing)
    crossing_pass_length_um = 2.0 * crossing_half_um
    entry_um = float(geometry.entry_straight_um)
    level_count = max(level for level, *_ in crossings) + 1
    usable_um = float(band_width_um) - 2.0 * float(geometry.band_margin_um)
    column_pitch_um = usable_um / float(level_count)
    x_left = -0.5 * usable_um
    x_right = 0.5 * usable_um
    min_pitch_um = 2.0 * (crossing_half_um / math.sqrt(2.0) + native_dx + 1.0)
    if column_pitch_um < min_pitch_um:
        raise ValueError(
            f"stretched crossing grid: {level_count} swap levels in a {band_width_um:.1f} um band "
            f"leave {column_pitch_um:.1f} um per level, below the {min_pitch_um:.1f} um a "
            "crossing with 45-degree arms needs"
        )

    def level_x(level: int) -> float:
        return x_left + (level + 0.5) * column_pitch_um

    component = Component(name=component_name)
    crossing_ref_by_key: dict[tuple[int, int], object] = {}
    crossing_key_by_lane_level: dict[tuple[str, int], tuple[int, int]] = {}
    for level, upper_slot, upper, lower in crossings:
        ref = component.add_ref(crossing)
        ref.rotate(-45.0)
        cx = level_x(level)
        cy = 0.5 * (rows[upper_slot] + rows[upper_slot + 1])
        center = ref.dbbox().center()
        ref.dmove((cx - float(center.x), cy - float(center.y)))
        crossing_ref_by_key[(level, upper_slot)] = ref
        crossing_key_by_lane_level[(upper.net_name, level)] = (level, upper_slot)
        crossing_key_by_lane_level[(lower.net_name, level)] = (level, upper_slot)

    def _bend_that_turns(from_heading: float, to_heading: float) -> Component:
        delta = (to_heading - from_heading + 180.0) % 360.0 - 180.0
        if abs(abs(delta) - 45.0) > 1e-6:
            raise RuntimeError(f"stretched grid only bends by 45 degrees, asked {delta}")
        return bend_ccw if delta > 0 else bend_cw

    def _bend_displacement(from_heading: float, to_heading: float) -> tuple[float, float]:
        delta = (to_heading - from_heading + 180.0) % 360.0 - 180.0
        native = (native_dx, native_dy if delta > 0 else -native_dy)
        return _rotate(native, from_heading)

    def _check(port_a, port_b, what: str) -> None:
        a, b = port_a.dcenter, port_b.dcenter
        gap = math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))
        if gap > _GEOMETRY_TOLERANCE_UM:
            raise RuntimeError(
                f"stretched crossing grid geometry mismatch ({what}): {tuple(a)} vs {tuple(b)}"
            )

    def _add_straight(open_port, length_um: float, what: str, net: str):
        if length_um < -_GEOMETRY_TOLERANCE_UM:
            raise ValueError(
                f"stretched crossing grid: negative {what} ({length_um:.3f} um) for lane {net}"
            )
        if length_um <= _GEOMETRY_TOLERANCE_UM:
            return open_port, 0.0
        ref = component.add_ref(_straight(geometry, length_um))
        ref.connect("o1", open_port)
        return ref.ports["o2"], float(length_um)

    def _add_bend(open_port, from_heading: float, to_heading: float):
        ref = component.add_ref(_bend_that_turns(from_heading, to_heading))
        ref.connect("o1", open_port)
        return ref.ports["o2"], bend_length_um

    def _unit(heading: float) -> tuple[float, float]:
        return (math.cos(math.radians(heading)), math.sin(math.radians(heading)))

    def _connect(
        open_port,
        from_heading: float,
        target_xy: tuple[float, float],
        to_heading: float,
        net: str,
        what: str,
    ):
        """Straight(s) and 45-degree bend(s) from ``open_port`` to ``target_xy``.

        Headings are 0 (east), +-45 (diagonals) or +-90 (vertical). Solves the
        straight lengths so the path ends exactly on ``target_xy`` with
        ``to_heading``. Same heading: one straight. Headings 45 degrees apart:
        straight, bend, straight. 90 degrees apart (diagonal to diagonal via
        horizontal or vertical): straight, bend, straight, bend, straight,
        with the two outer straights equal.
        """
        start = (float(open_port.dcenter[0]), float(open_port.dcenter[1]))
        dx = target_xy[0] - start[0]
        dy = target_xy[1] - start[1]
        u = _unit(from_heading)
        v = _unit(to_heading)
        length = 0.0
        delta = (to_heading - from_heading + 180.0) % 360.0 - 180.0
        if abs(delta) < 1e-6:
            along = dx * u[0] + dy * u[1]
            off = abs(-dx * u[1] + dy * u[0])
            if off > _GEOMETRY_TOLERANCE_UM:
                raise ValueError(
                    f"stretched crossing grid: {what} for lane {net} is off-axis by {off:.3f} um"
                )
            return _add_straight(open_port, along, what, net)
        if abs(abs(delta) - 45.0) < 1e-6:
            b = _bend_displacement(from_heading, to_heading)
            # a * u + b + c * v = (dx, dy)  -> solve the 2x2 system for a, c
            det = u[0] * v[1] - u[1] * v[0]
            rx, ry = dx - b[0], dy - b[1]
            a = (rx * v[1] - ry * v[0]) / det
            c = (u[0] * ry - u[1] * rx) / det
            if a < -_GEOMETRY_TOLERANCE_UM and from_heading == 0.0:
                # The crossing is too close in x for one 45-degree arm across
                # the row gap: bend at once and let the diagonal-to-diagonal
                # connector insert a vertical run.
                open_port, run = _add_bend(open_port, from_heading, to_heading)
                length += run
                open_port, run = _connect_via(open_port, to_heading, target_xy, net, what)
                return open_port, length + run
            if c < -_GEOMETRY_TOLERANCE_UM and to_heading == 0.0:
                # Mirror image on the exit side: vertical run first, then the
                # final bend onto the row lands exactly at the target.
                pre_bend = (target_xy[0] - b[0], target_xy[1] - b[1])
                open_port, run = _connect_via(open_port, from_heading, pre_bend, net, what)
                length += run
                open_port, run = _add_bend(open_port, from_heading, to_heading)
                return open_port, length + run
            open_port, run = _add_straight(open_port, a, f"{what} (before bend)", net)
            length += run
            open_port, run = _add_bend(open_port, from_heading, to_heading)
            length += run
            open_port, run = _add_straight(open_port, c, f"{what} (after bend)", net)
            length += run
            return open_port, length
        if abs(abs(delta) - 90.0) < 1e-6:
            # diagonal -> diagonal (same diagonal heading on both ends is the
            # 0-degree case above); here from_heading == to_heading is
            # impossible, so this branch is diagonal -> opposite... not used.
            raise RuntimeError(
                "stretched crossing grid: 90-degree connector is built by _connect_via"
            )
        raise RuntimeError(f"stretched crossing grid: unsupported turn {delta} for lane {net}")

    def _connect_via(
        open_port, heading: float, target_xy: tuple[float, float], net: str, what: str
    ):
        """Diagonal (``heading`` = +-45) to the same diagonal heading at ``target_xy``.

        Straight diagonal when the displacement lies on the diagonal;
        otherwise arm, bend, horizontal or vertical run, bend, arm with
        equal arms.
        """
        start = (float(open_port.dcenter[0]), float(open_port.dcenter[1]))
        dx = target_xy[0] - start[0]
        dy = target_xy[1] - start[1]
        u = _unit(heading)
        if abs(abs(dx) - abs(dy)) <= _GEOMETRY_TOLERANCE_UM:
            return _add_straight(open_port, dx * u[0] + dy * u[1], what, net)
        via = 0.0 if abs(dy) < abs(dx) else (90.0 if dy > 0 else -90.0)
        b1 = _bend_displacement(heading, via)
        b2 = _bend_displacement(via, heading)
        w = _unit(via)
        # a*u + b1 + m*w + b2 + a*u = (dx, dy): two unknowns a, m
        rx, ry = dx - b1[0] - b2[0], dy - b1[1] - b2[1]
        det = 2.0 * u[0] * w[1] - 2.0 * u[1] * w[0]
        a = (rx * w[1] - ry * w[0]) / det
        m = (2.0 * u[0] * ry - 2.0 * u[1] * rx) / det
        length = 0.0
        open_port, run = _add_straight(open_port, a, f"{what} (arm out)", net)
        length += run
        open_port, run = _add_bend(open_port, heading, via)
        length += run
        run_kind = "horizontal" if via == 0.0 else "vertical"
        open_port, run = _add_straight(open_port, m, f"{what} ({run_kind} run)", net)
        length += run
        open_port, run = _add_bend(open_port, via, heading)
        length += run
        open_port, run = _add_straight(open_port, a, f"{what} (arm in)", net)
        length += run
        return open_port, length

    lane_length_um: dict[str, float] = {}
    output_port_by_net: dict[str, str] = {}
    for start_slot, edge in enumerate(lanes):
        net = edge.net_name
        first, last, direction = movement[net]
        heading = -45.0 if direction > 0 else 45.0
        enter_port, exit_port = ("o1", "o3") if direction > 0 else ("o4", "o2")
        length = 0.0
        ref = component.add_ref(_straight(geometry, entry_um))
        ref.dmove((x_left, rows[start_slot]))
        component.add_port(f"in_{start_slot}", port=ref.ports["o1"])
        open_port = ref.ports["o2"]
        length += entry_um
        current_heading = 0.0
        for level in range(first, last + 1):
            x_ref = crossing_ref_by_key[crossing_key_by_lane_level[(net, level)]]
            target = x_ref.ports[enter_port].dcenter
            target_xy = (float(target[0]), float(target[1]))
            if current_heading == 0.0:
                open_port, run = _connect(
                    open_port, 0.0, target_xy, heading, net, f"approach to level {level}"
                )
            else:
                open_port, run = _connect_via(
                    open_port, heading, target_xy, net, f"link level {level - 1} -> {level}"
                )
            length += run
            _check(open_port, x_ref.ports[enter_port], f"lane {net} level {level} entry")
            open_port = x_ref.ports[exit_port]
            length += crossing_pass_length_um
            current_heading = heading
        end_slot = final_slot_by_net[net]
        exit_row = rows[end_slot]
        exit_stub_x = x_right - entry_um
        open_port, run = _connect(open_port, heading, (exit_stub_x, exit_row), 0.0, net, "exit")
        length += run
        ref = component.add_ref(_straight(geometry, entry_um))
        ref.dmove((exit_stub_x, exit_row))
        _check(ref.ports["o1"], open_port, f"lane {net} exit stub")
        component.add_port(f"out_{end_slot}", port=ref.ports["o2"])
        output_port_by_net[net] = f"out_{end_slot}"
        lane_length_um[net] = length + entry_um

    input_port_by_net = {edge.net_name: f"in_{index}" for index, edge in enumerate(lanes)}
    component.info["crossing_count"] = len(crossings)
    component.info["lane_count"] = lane_count
    component.info["level_count"] = level_count
    component.info["column_pitch_um"] = column_pitch_um
    result = CrossingGridBuildResult(
        component=component,
        crossing_count=len(crossings),
        lane_length_um_by_net_name=lane_length_um,
        input_port_name_by_net_name=input_port_by_net,
        output_port_name_by_net_name=output_port_by_net,
        width_um=usable_um,
        height_um=(max(rows) - min(rows)) + 2.0 * crossing_half_um,
    )
    _GRID_BUILD_CACHE[component_name] = result
    return result


# ---------------------------------------------------------------------------
# Schematic / layout derivation
# ---------------------------------------------------------------------------


def crossing_grid_geometry_from_env() -> CrossingGridGeometry:
    """Default grid geometry, with environment-variable overrides for experiments.

    ``PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM``,
    ``PHOTONIC_ROUTER_CROSSING_GRID_BEND_RADIUS_UM`` and
    ``PHOTONIC_ROUTER_CROSSING_GRID_ENTRY_STRAIGHT_UM`` override the
    corresponding ``CrossingGridGeometry`` fields when set.
    """
    import os

    base = CrossingGridGeometry()

    def _read(name: str, default: float) -> float:
        raw = os.environ.get(name, "").strip()
        if not raw:
            return default
        try:
            return float(raw)
        except ValueError as exc:
            raise ValueError(f"{name} must be a number, got {raw!r}") from exc

    return CrossingGridGeometry(
        lane_pitch_um=_read("PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM", base.lane_pitch_um),
        bend_radius_um=_read("PHOTONIC_ROUTER_CROSSING_GRID_BEND_RADIUS_UM", base.bend_radius_um),
        entry_straight_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_ENTRY_STRAIGHT_UM", base.entry_straight_um
        ),
        fan_column_pitch_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_FAN_COLUMN_PITCH_UM", base.fan_column_pitch_um
        ),
        port_pair_spread_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_PORT_PAIR_SPREAD_UM", base.port_pair_spread_um
        ),
        unrouted_sibling_clearance_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_UNROUTED_SIBLING_CLEARANCE_UM",
            base.unrouted_sibling_clearance_um,
        ),
        fan_mode=os.environ.get("PHOTONIC_ROUTER_CROSSING_GRID_FAN_MODE", base.fan_mode).strip()
        or base.fan_mode,
        stub_stagger_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_STUB_STAGGER_UM", base.stub_stagger_um
        ),
        slot_spread_um=_read("PHOTONIC_ROUTER_CROSSING_GRID_SLOT_SPREAD_UM", base.slot_spread_um),
        band_margin_um=_read("PHOTONIC_ROUTER_CROSSING_GRID_BAND_MARGIN_UM", base.band_margin_um),
        tile_placement=os.environ.get(
            "PHOTONIC_ROUTER_CROSSING_GRID_TILE_PLACEMENT", base.tile_placement
        ).strip()
        or base.tile_placement,
        tile_min_spacing_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_TILE_MIN_SPACING_UM", base.tile_min_spacing_um
        ),
        column_pitch_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_PITCH_UM", base.column_pitch_um
        ),
        column_lead_um=_read("PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_LEAD_UM", base.column_lead_um),
        corner_margin_um=_read(
            "PHOTONIC_ROUTER_CROSSING_GRID_CORNER_MARGIN_UM", base.corner_margin_um
        ),
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


@dataclass(frozen=True)
class _RunPlacement:
    x_center_um: float
    y_center_um: float
    entry_row_by_net: dict[str, float]  # relative to y_center_um
    exit_row_by_net: dict[str, float]
    band_width_um: float = 0.0  # free x between the layer's source and target ports


def _spread_sibling_rows(
    rows_by_net: Mapping[str, float],
    instance_by_net: Mapping[str, str],
    spread_um: float,
) -> dict[str, float]:
    """Push ports that share a component instance apart symmetrically.

    A switch's two ports are only 1.25 um apart -- less than one routing cell --
    so the grid face spreads siblings to ``2 * spread_um`` so each stub lands
    in its own cell row.
    """
    by_instance: dict[str, list[str]] = {}
    for net, instance in instance_by_net.items():
        by_instance.setdefault(instance, []).append(net)
    result = dict(rows_by_net)
    for nets in by_instance.values():
        if len(nets) < 2:
            continue
        ordered = sorted(nets, key=lambda net: rows_by_net[net])
        center = sum(rows_by_net[net] for net in ordered) / float(len(ordered))
        for index, net in enumerate(ordered):
            result[net] = center + (index - 0.5 * (len(ordered) - 1)) * 2.0 * spread_um
    return result


def _push_away_from_unrouted_siblings(
    rows_by_net: Mapping[str, float],
    port_spec_by_net: Mapping[str, str],
    unrouted_layout: Component,
    clearance_um: float,
) -> dict[str, float]:
    """Keep every grid port ``clearance_um`` away from same-instance ports not in the grid.

    A lane that crosses nothing is routed as an ordinary net along its natural
    row; if a sibling lane's grid port sat 1.25 um from that row, the grid's
    bounding box would block it and force a detour around the grid.
    """
    in_grid = set(port_spec_by_net.values())
    result = dict(rows_by_net)
    for net, spec in port_spec_by_net.items():
        instance_name, port_name = spec.split(",")
        own = get_port_from_instance(unrouted_layout, instance_name, port_name)
        own_y = float(own.dcenter[1])
        row = result[net]
        for other in unrouted_layout.insts[instance_name].ports:
            other_spec = f"{instance_name},{other.name}"
            if other_spec == spec or other_spec in in_grid:
                continue
            if abs(float(other.orientation) - float(own.orientation)) % 360.0 > 1e-6:
                continue
            other_y = float(other.dcenter[1])
            gap = row - other_y
            if abs(gap) >= clearance_um:
                continue
            direction = 1.0 if (gap > 0 or (gap == 0 and own_y >= other_y)) else -1.0
            row = other_y + direction * clearance_um
        result[net] = row
    return result


def _run_placement(
    unrouted_layout: Component,
    schematic: Schematic,
    lanes: Iterable[RoutedEdgeKey],
    geometry: CrossingGridGeometry,
) -> _RunPlacement:
    """Where a run's grid goes and where each lane enters/leaves it.

    The grid is centered horizontally in the free band between the layer's
    source ports and target ports, and vertically on the mean of all lane
    rows. Entry/exit rows are the natural source/target port rows (siblings
    spread apart), relative to that vertical center.
    """
    source_xs: list[float] = []
    target_xs: list[float] = []
    entry_rows: dict[str, float] = {}
    exit_rows: dict[str, float] = {}
    source_instance: dict[str, str] = {}
    target_instance: dict[str, str] = {}
    source_spec: dict[str, str] = {}
    target_spec: dict[str, str] = {}
    for edge in lanes:
        p1, p2 = _net_endpoints(schematic, edge.net_name)
        inst1, port1 = p1.split(",")
        inst2, port2 = p2.split(",")
        source_spec[edge.net_name] = p1
        target_spec[edge.net_name] = p2
        source = get_port_from_instance(unrouted_layout, inst1, port1)
        target = get_port_from_instance(unrouted_layout, inst2, port2)
        source_xs.append(float(source.dcenter[0]))
        target_xs.append(float(target.dcenter[0]))
        entry_rows[edge.net_name] = float(source.dcenter[1])
        exit_rows[edge.net_name] = float(target.dcenter[1])
        source_instance[edge.net_name] = inst1
        target_instance[edge.net_name] = inst2
    entry_rows = _spread_sibling_rows(entry_rows, source_instance, geometry.port_pair_spread_um)
    exit_rows = _spread_sibling_rows(exit_rows, target_instance, geometry.port_pair_spread_um)
    entry_rows = _push_away_from_unrouted_siblings(
        entry_rows, source_spec, unrouted_layout, geometry.unrouted_sibling_clearance_um
    )
    exit_rows = _push_away_from_unrouted_siblings(
        exit_rows, target_spec, unrouted_layout, geometry.unrouted_sibling_clearance_um
    )
    if geometry.fan_mode == "router":
        # Ports sit on the lattice slot rows; the router does the fan-out.
        all_rows = list(entry_rows.values()) + list(exit_rows.values())
        return _RunPlacement(
            x_center_um=round(0.5 * (max(source_xs) + min(target_xs)), 3),
            y_center_um=round(sum(all_rows) / float(len(all_rows)), 3),
            entry_row_by_net={},
            exit_row_by_net={},
            band_width_um=float(min(target_xs) - max(source_xs)),
        )
    if geometry.fan_mode == "stretched":
        # Slot rows are the natural port rows with each switch's sibling pair
        # spread to +-slot_spread_um; the component spans the band.
        natural_entry = {
            net: float(get_port_from_instance(unrouted_layout, *spec.split(",")).dcenter[1])
            for net, spec in source_spec.items()
        }
        natural_exit = {
            net: float(get_port_from_instance(unrouted_layout, *spec.split(",")).dcenter[1])
            for net, spec in target_spec.items()
        }
        slot_entry = _spread_sibling_rows(natural_entry, source_instance, geometry.slot_spread_um)
        slot_exit = _spread_sibling_rows(natural_exit, target_instance, geometry.slot_spread_um)
        x_center = round(0.5 * (max(source_xs) + min(target_xs)), 3)
        y_center = round(sum(slot_entry.values()) / float(len(slot_entry)), 3)
        return _RunPlacement(
            x_center_um=x_center,
            y_center_um=y_center,
            entry_row_by_net={net: row - y_center for net, row in slot_entry.items()},
            exit_row_by_net={net: row - y_center for net, row in slot_exit.items()},
            band_width_um=float(min(target_xs) - max(source_xs)),
        )
    if geometry.fan_mode != "in_grid":
        raise ValueError(
            f"unknown fan_mode {geometry.fan_mode!r}; use 'router', 'in_grid', 'stretched' "
            "or 'tiles'"
        )
    # Snap the placement to the 1 nm database unit so a grid port lands on
    # *exactly* the same coordinate as the switch port it faces: the endpoint
    # corrector only uses the plain "shift the straight" strategy when the two
    # port rows agree exactly, and a 1 nm mismatch would cost a 12 um bump.
    x_center = round(0.5 * (max(source_xs) + min(target_xs)), 3)
    all_rows = list(entry_rows.values()) + list(exit_rows.values())
    y_center = round(sum(all_rows) / float(len(all_rows)), 3)
    return _RunPlacement(
        x_center_um=x_center,
        y_center_um=y_center,
        entry_row_by_net={net: row - y_center for net, row in entry_rows.items()},
        exit_row_by_net={net: row - y_center for net, row in exit_rows.items()},
    )


def split_stage_into_participating_runs(
    stage_plan: CrossingStagePlan,
) -> list[CrossingStagePlan]:
    """Split a layer into maximal contiguous runs of lanes that cross something.

    A lane that takes part in no crossing of the layer is never passed by any
    other lane (every event is an adjacent swap), so it partitions the lanes:
    the crossing schedule of the lanes above it and the lanes below it are
    independent. Each run becomes its own grid; non-participating lanes are
    left to ordinary routing. Event levels are renumbered per run so a run's
    lattice is as narrow as its own schedule allows.
    """
    participating = {event.edge_a for event in stage_plan.events} | {
        event.edge_b for event in stage_plan.events
    }
    runs: list[list[RoutedEdgeKey]] = []
    current: list[RoutedEdgeKey] = []
    for edge in stage_plan.initial_edge_order:
        if edge in participating:
            current.append(edge)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)

    from dataclasses import replace

    result: list[CrossingStagePlan] = []
    for run in runs:
        run_set = set(run)
        events = [event for event in stage_plan.events if event.edge_a in run_set]
        levels = sorted({int(event.level) for event in events})
        level_map = {level: index for index, level in enumerate(levels)}
        renumbered = tuple(replace(event, level=level_map[int(event.level)]) for event in events)
        run_plan = CrossingStagePlan(
            source_depth=stage_plan.source_depth,
            target_depth=stage_plan.target_depth,
            initial_edge_order=tuple(run),
            final_edge_order=tuple(edge for edge in stage_plan.final_edge_order if edge in run_set),
            events=renumbered,
        )
        run_plan.validate()
        result.append(run_plan)
    return result


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


def _derive_crossing_tiles(
    schematic: Schematic,
    unrouted_layout: Component,
    crossing_plan: CrossingPlan,
    geometry: CrossingGridGeometry,
    derived: Schematic,
    router_probe_kwargs: Mapping[str, object] | None = None,
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
    tile = crossing_tile_component(geometry)
    _register_grid_cell(tile.name, tile)
    if geometry.tile_placement == "lines":
        return _derive_crossing_tiles_on_lines(
            schematic, unrouted_layout, crossing_plan, geometry, derived, tile
        )
    if geometry.tile_placement not in ("columns", "auto", "column-grid"):
        raise ValueError(
            f"unknown tile_placement {geometry.tile_placement!r}; use 'auto', 'columns', "
            "'column-grid' or 'lines'"
        )
    anchors: dict[str, tuple[float, float]] = {}
    if geometry.tile_placement in ("auto", "column-grid"):
        import os

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
            if geometry.tile_placement != "column-grid" and evaluate_x_array(
                schematic, unrouted_layout, stage_plan, geometry
            ).feasible:
                continue
            count: dict[str, int] = {}
            for edge in stage_plan.initial_edge_order:
                count[edge.source.instance] = count.get(edge.source.instance, 0) + 1
            dense.update(name for name, n in count.items() if n >= 2)
        if dense:
            os.environ["PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES"] = ",".join(sorted(dense))
        else:
            os.environ.pop("PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES", None)
        anchors = static_fanout_anchors_um(
            unrouted_layout, schematic, **dict(router_probe_kwargs or {})
        )
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

        allowed = {
            "column-grid": (COLUMN_GRID,),
            "columns": (X_ARRAY,),
            "auto": (X_ARRAY, COLUMN_GRID),
        }[geometry.tile_placement]
        decision = select_layer_structure(
            schematic, unrouted_layout, stage_plan, geometry, anchors,
            stage_key=tuple(stage_key), allowed=allowed,
        )
        decisions.append(decision)
        print(f"      - crossing structure {decision.describe()}")
        if decision.chosen == ROUTER:
            fallback_layers.append(decision)
            continue
        if decision.chosen == COLUMN_GRID:
            stage_result = _column_grid_stage(
                schematic, unrouted_layout, stage_plan, geometry, derived, plus_tile, anchors
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
        for level, upper_slot, upper, lower in crossings:
            name = f"{CROSSING_TILE_PREFIX}stage{depth}_l{level}_s{upper_slot}"
            x = band_x0 + float(geometry.band_margin_um) + (level + 0.5) * pitch
            y = 0.5 * (rows[upper_slot] + rows[upper_slot + 1])
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
    if fallback_layers:
        # Step 2 of the builder plan hands these layers to the guided router;
        # until then a layer without a feasible alignment stops the run.
        raise ValueError(
            "no pre-placed crossing structure fits: "
            + " | ".join(d.describe() for d in fallback_layers)  # type: ignore[attr-defined]
        )
    return tile_names, builds, stub_nets, lengths, nets, split, expected, placed, decisions


def _line_intersection(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> tuple[float, float] | None:
    """Intersection of the segments a0-a1 and b0-b1 (None if parallel or outside)."""
    dax, day = a1[0] - a0[0], a1[1] - a0[1]
    dbx, dby = b1[0] - b0[0], b1[1] - b0[1]
    den = dax * dby - day * dbx
    if abs(den) < 1e-12:
        return None
    t = ((b0[0] - a0[0]) * dby - (b0[1] - a0[1]) * dbx) / den
    u = ((b0[0] - a0[0]) * day - (b0[1] - a0[1]) * dax) / den
    if t < -1e-9 or t > 1.0 + 1e-9 or u < -1e-9 or u > 1.0 + 1e-9:
        return None
    return (a0[0] + t * dax, a0[1] + t * day)


def _derive_crossing_tiles_on_lines(
    schematic: Schematic,
    unrouted_layout: Component,
    crossing_plan: CrossingPlan,
    geometry: CrossingGridGeometry,
    derived: Schematic,
    x_tile: Component,
) -> tuple[
    list[str],
    dict[str, CrossingGridBuildResult],
    dict[str, tuple[str, str]],
    dict[str, float],
    list[Net],
    set[str],
    int,
    int,
]:
    """Tiles at the intersections of the crossing lanes' straight lines.

    Every lane of a stage is drawn as the straight line from its source port
    to its target port. Two lanes whose order flips between the two layers
    cross exactly once on those lines, and that point is where the planned
    crossing's tile goes: an "X" tile when the lanes move in opposite
    vertical directions, a "+" tile (one lane horizontal, the steeper lane
    vertical) otherwise. Tiles closer than ``tile_min_spacing_um`` are pushed
    apart in a few relaxation passes, and every tile keeps
    ``band_margin_um`` from the source and target port columns. A lane's
    tiles are ordered along its line, and the lane becomes router nets
    between them (``__to_grid``, ``__via<i>``, ``__from_grid``).
    """
    plus_tile = crossing_tile_component(geometry, axis_aligned=True)
    _register_grid_cell(plus_tile.name, plus_tile)
    crossing = _crossing_component()
    pass_length_um = 2.0 * _crossing_half_extent_um(crossing)
    min_sep = float(geometry.tile_min_spacing_um)
    tile_names: list[str] = []
    builds: dict[str, CrossingGridBuildResult] = {}
    stub_nets: dict[str, tuple[str, str]] = {}
    lengths: dict[str, float] = {}
    nets: list[Net] = []
    split: set[str] = set()
    expected = 0
    placed = 0
    for stage_key in sorted(crossing_plan.stages):
        stage_plan = crossing_plan.stages[stage_key]
        if not stage_plan.initial_edge_order or not stage_plan.events:
            continue
        depth = stage_plan.source_depth
        line: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {}
        endpoints: dict[str, tuple[str, str]] = {}
        for edge in stage_plan.initial_edge_order:
            p1, p2 = _net_endpoints(schematic, edge.net_name)
            source = get_port_from_instance(unrouted_layout, *p1.split(","))
            target = get_port_from_instance(unrouted_layout, *p2.split(","))
            line[edge.net_name] = (
                (float(source.dcenter[0]), float(source.dcenter[1])),
                (float(target.dcenter[0]), float(target.dcenter[1])),
            )
            endpoints[edge.net_name] = (p1, p2)
        band_x0 = max(ln[0][0] for ln in line.values()) + float(geometry.band_margin_um)
        band_x1 = min(ln[1][0] for ln in line.values()) - float(geometry.band_margin_um)
        # one tile per planned pair
        tiles: list[dict] = []
        for event in stage_plan.events:
            a, b = event.edge_a.net_name, event.edge_b.net_name
            point = _line_intersection(*line[a], *line[b])
            if point is None:
                # parallel or touching lines (e.g. both endpoints on the same
                # rows): fall back to the four-port average
                pts = [line[a][0], line[a][1], line[b][0], line[b][1]]
                point = (sum(p[0] for p in pts) / 4.0, sum(p[1] for p in pts) / 4.0)
            dya = line[a][1][1] - line[a][0][1]
            dyb = line[b][1][1] - line[b][0][1]
            opposite = dya * dyb < -1e-9
            if opposite:
                kind = "x"
                roles = {a: "down" if dya < 0 else "up", b: "down" if dyb < 0 else "up"}
            else:
                kind = "plus"
                steep, flat = (a, b) if abs(dya) >= abs(dyb) else (b, a)
                dys = line[steep][1][1] - line[steep][0][1]
                roles = {flat: "flat", steep: "vert_up" if dys > 0 else "vert_down"}
            x = min(max(point[0], band_x0), band_x1)
            tiles.append({"pair": (a, b), "x": x, "y": point[1], "kind": kind, "roles": roles})
        # relaxation: push tiles apart to min_sep (moves along the joining vector)
        for _ in range(60):
            moved = False
            for i in range(len(tiles)):
                for j in range(i + 1, len(tiles)):
                    dx = tiles[j]["x"] - tiles[i]["x"]
                    dy = tiles[j]["y"] - tiles[i]["y"]
                    d = math.hypot(dx, dy)
                    if d >= min_sep - 1e-9:
                        continue
                    if d < 1e-9:
                        dx, dy, d = 1.0, 0.0, 1.0
                    push = 0.5 * (min_sep - d) / d
                    tiles[i]["x"] -= dx * push
                    tiles[i]["y"] -= dy * push
                    tiles[j]["x"] += dx * push
                    tiles[j]["y"] += dy * push
                    moved = True
            for t in tiles:
                t["x"] = min(max(t["x"], band_x0), band_x1)
            if not moved:
                break
        # place
        tiles_of_lane: dict[str, list[dict]] = {}
        for index, t in enumerate(tiles):
            comp = x_tile if t["kind"] == "x" else plus_tile
            name = f"{CROSSING_TILE_PREFIX}stage{depth}_{index}"
            derived.add_instance(
                name,
                Instance(component=comp.name),
                Placement(x=round(t["x"], 3), y=round(t["y"], 3)),
            )
            t["name"] = name
            tile_names.append(name)
            placed += 1
            for lane in t["pair"]:
                tiles_of_lane.setdefault(lane, []).append(t)
        expected += len(stage_plan.events)
        rows = [ln[0][1] for ln in line.values()]
        builds[f"stage{depth}"] = CrossingGridBuildResult(
            component=x_tile,
            crossing_count=len(tiles),
            lane_length_um_by_net_name={},
            input_port_name_by_net_name={},
            output_port_name_by_net_name={},
            width_um=band_x1 - band_x0,
            height_um=max(rows) - min(rows),
        )
        # nets: along each lane's line, ordered by x (the line is monotone in x)
        for lane, lane_tiles in tiles_of_lane.items():
            lane_tiles.sort(key=lambda t: t["x"])
            p1, p2 = endpoints[lane]
            previous = p1
            names: list[str] = []
            for index, t in enumerate(lane_tiles):
                role = t["roles"][lane]
                enter, leave = {
                    "down": ("ul", "lr"),
                    "up": ("ll", "ur"),
                    "flat": ("l", "r"),
                    "vert_up": ("b", "t"),
                    "vert_down": ("t", "b"),
                }[role]
                if index == 0:
                    segment = f"{lane}{TO_GRID_SUFFIX}"
                else:
                    segment = f"{lane}{VIA_GRID_SUFFIX}{index}"
                nets.append(Net(p1=previous, p2=f"{t['name']},{enter}", name=segment))
                names.append(segment)
                previous = f"{t['name']},{leave}"
            final = f"{lane}{FROM_GRID_SUFFIX}"
            nets.append(Net(p1=previous, p2=p2, name=final))
            names.append(final)
            stub_nets[lane] = (names[0], names[-1])
            lengths[lane] = pass_length_um * len(lane_tiles)
            split.add(lane)
    return tile_names, builds, stub_nets, lengths, nets, split, expected, placed


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
    import os

    if os.environ.get("PHOTONIC_ROUTER_TRACE_COLUMN_GRID"):
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
    import os as _os

    # "always" (default): both corners of every moving lane are pre-wired
    # (crossing-with-bend tile or bare bend), so the router only routes
    # straight runs and no lane can stray across another lane's column
    # (multiportmmi_32x32: a tile-less lane routed diagonally through a
    # column). "auto": only corners whose legs are shorter than
    # corner_margin_um. "router": none.
    corners_mode = (
        _os.environ.get("PHOTONIC_ROUTER_CROSSING_GRID_CORNERS", "always").strip() or "always"
    )
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

    geometry = geometry or crossing_grid_geometry_from_env()
    derived = Schematic()
    for instance_name, instance in schematic.netlist.instances.items():
        derived.add_instance(instance_name, instance, schematic.placements[instance_name])

    interstage_net_names: set[str] = set()
    grid_instance_names: list[str] = []
    grid_builds: dict[str, CrossingGridBuildResult] = {}
    stub_nets: dict[str, tuple[str, str]] = {}
    lengths: dict[str, float] = {}
    to_grid_nets: list[Net] = []
    from_grid_nets: list[Net] = []
    expected_crossings = 0
    placed_crossings = 0

    if geometry.fan_mode == "tiles":
        tiles_result = _derive_crossing_tiles(
            schematic, unrouted_layout, crossing_plan, geometry, derived, router_probe_kwargs
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
        )

    for stage_key in sorted(crossing_plan.stages):
        stage_plan = crossing_plan.stages[stage_key]
        if not stage_plan.initial_edge_order or not stage_plan.events:
            # Layers without any crossing (e.g. IO-to-switch) keep their
            # ordinary single nets; a grid would add nothing.
            continue
        for run_index, run_plan in enumerate(split_stage_into_participating_runs(stage_plan)):
            placement = _run_placement(
                unrouted_layout, schematic, run_plan.initial_edge_order, geometry
            )
            build = build_crossing_grid_component(
                run_plan,
                geometry,
                entry_row_by_net=placement.entry_row_by_net,
                exit_row_by_net=placement.exit_row_by_net,
                band_width_um=placement.band_width_um,
            )
            instance_name = f"{GRID_INSTANCE_PREFIX}{stage_plan.source_depth}_run{run_index}"
            _register_grid_cell(build.component.name, build.component)
            derived.add_instance(
                instance_name,
                Instance(component=build.component.name),
                Placement(x=placement.x_center_um, y=placement.y_center_um),
            )
            grid_instance_names.append(instance_name)
            grid_builds[instance_name] = build
            expected_crossings += len(run_plan.events)
            placed_crossings += build.crossing_count

            for edge in run_plan.initial_edge_order:
                net_name = edge.net_name
                p1, p2 = _net_endpoints(schematic, net_name)
                in_port = build.input_port_name_by_net_name[net_name]
                out_port = build.output_port_name_by_net_name[net_name]
                to_name = f"{net_name}{TO_GRID_SUFFIX}"
                from_name = f"{net_name}{FROM_GRID_SUFFIX}"
                to_grid_nets.append(Net(p1=p1, p2=f"{instance_name},{in_port}", name=to_name))
                from_grid_nets.append(Net(p1=f"{instance_name},{out_port}", p2=p2, name=from_name))
                stub_nets[net_name] = (to_name, from_name)
                lengths[net_name] = build.lane_length_um_by_net_name[net_name]
                interstage_net_names.add(net_name)

    # Preserve the original net order for everything that is not split, and
    # insert the two stubs where the original net used to be.
    for net_name in schematic.netlist.routes:
        if net_name in interstage_net_names:
            continue
        p1, p2 = _net_endpoints(schematic, net_name)
        derived.add_net(Net(p1=p1, p2=p2, name=net_name))
    for net in to_grid_nets:
        derived.add_net(net)
    for net in from_grid_nets:
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
