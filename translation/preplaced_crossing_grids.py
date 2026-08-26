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

    lane_pitch_um: float = 20.0
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
    fan_mode: str = "router"
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
    x_left = core_x0 - entry_fan_um - entry_um
    x_right = core_x0 + core_width_um + exit_fan_um + entry_um
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

        # Entry straight + port at the lane's natural row.
        ref = component.add_ref(_straight(geometry, entry_um))
        ref.dmove((x_left, entry_rows[net]))
        component.add_port(f"in_{start_slot}", port=ref.ports["o1"])
        open_port = ref.ports["o2"]
        length += entry_um

        # Fan-in: turn in this lane's own column, land on the slot row.
        column = entry_columns.get(net)
        if column is not None:
            open_port, run = _straight_to_x(
                open_port, x_left + entry_um + column * column_pitch_um, net, "entry fan"
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
        open_port, run = _straight_to_x(open_port, x_right - entry_um, net, "exit")
        length += run
        # Place the exit straight absolutely (like the entry straight) rather
        # than chaining it: port-to-port connects accumulate ~1 nm of dbu
        # rounding, and the exit port must sit on *exactly* the row the next
        # switch port has, or the stub's endpoint corrector falls back from a
        # plain straight shift to a 12 um bump.
        ref = component.add_ref(_straight(geometry, entry_um))
        ref.dmove((x_right - entry_um, exit_rows[net]))
        _check(ref.ports["o1"], open_port, f"lane {net} exit straight")
        component.add_port(f"out_{end_slot}", port=ref.ports["o2"])
        output_port_by_net[net] = f"out_{end_slot}"
        lane_length_um[net] = length + entry_um

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
        route_width_um=base.route_width_um,
        cross_section=base.cross_section,
    )


def _net_endpoints(schematic: Schematic, net_name: str) -> tuple[str, str]:
    bundle = schematic.netlist.routes[net_name]
    links = dict(bundle.links)
    if len(links) != 1:
        raise ValueError(f"net {net_name} must have exactly one link, has {len(links)}")
    (p1, p2), = links.items()
    return str(p1), str(p2)


@dataclass(frozen=True)
class _RunPlacement:
    x_center_um: float
    y_center_um: float
    entry_row_by_net: dict[str, float]  # relative to y_center_um
    exit_row_by_net: dict[str, float]


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
        )
    if geometry.fan_mode != "in_grid":
        raise ValueError(f"unknown fan_mode {geometry.fan_mode!r}; use 'router' or 'in_grid'")
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
        renumbered = tuple(
            replace(event, level=level_map[int(event.level)]) for event in events
        )
        run_plan = CrossingStagePlan(
            source_depth=stage_plan.source_depth,
            target_depth=stage_plan.target_depth,
            initial_edge_order=tuple(run),
            final_edge_order=tuple(
                edge for edge in stage_plan.final_edge_order if edge in run_set
            ),
            events=renumbered,
        )
        run_plan.validate()
        result.append(run_plan)
    return result


def _register_grid_cell(name: str, component: Component) -> None:
    # A grid port's approach is already protected by the grid's own committed
    # geometry (same reasoning as for static stubs), so it gets the smallest
    # possible access opening and no lane reservation -- otherwise the
    # reservation (+-4 cells) covers the natural row of a sibling lane that
    # crosses nothing and forces it to detour around the grid.
    register_component_port_access_rule(
        ComponentPortAccessRule(
            component_name_pattern=name,
            port_names=tuple(port.name for port in component.ports),
            access_length_um=0.0,
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


def derive_preplaced_crossing_layout(
    schematic: Schematic,
    unrouted_layout: Component,
    crossing_plan: CrossingPlan,
    *,
    geometry: CrossingGridGeometry | None = None,
) -> DerivedCrossingLayout:
    """Derive a schematic/layout pair with crossing grids placed and nets split.

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
                from_grid_nets.append(
                    Net(p1=f"{instance_name},{out_port}", p2=p2, name=from_name)
                )
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
