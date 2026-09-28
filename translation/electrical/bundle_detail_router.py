"""Detailed centerline routing for topology-derived individual bundles."""

from __future__ import annotations

import math
from collections import deque
from heapq import heappop, heappush
from typing import Literal, cast

from .individual_topology import bundle_route_side
from .metal_realization import common_bus_escape_rects
from .pad_slots import pad_access_cells
from .pitch_grid import bbox_to_grid_cells
from .terminal_contacts import _orientation_distance
from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalPortAccess,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    EscapeBundle,
    EscapeTopologyRoute,
    GridCell,
    GridPoint,
    IndividualEscapeTopologyResult,
    PadAssignment,
    PadPlan,
)

Axis = Literal["x", "y"]
RouteSide = Literal["left", "right"]
DirectionIndex = int
SearchState = tuple[GridCell, DirectionIndex]

_NO_DIRECTION = -1
_GRID_DIRECTIONS: tuple[GridCell, ...] = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
)


def route_detailed_bundles(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult,
    topology: IndividualEscapeTopologyResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> DetailedBundleRoutingResult:
    """Route every individual pad wire as a two-segment L, or a Z fallback.

    Two passes: pass 1 tries every wire as an L (``_l_shaped_pad_wire``),
    bundles in pad-row order and, inside a bundle, the terminal nearest the
    pad row first, committing each success's reservation footprint before
    the next attempt. Pass 2 gives every terminal pass 1 refused (wrong side
    or a blocked column) a full-grid A* fallback that must avoid every L's
    committed footprint, so a fallback wire can never cross an L wire.
    """

    config.validate()
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    track_pitch_cells = max(1, math.ceil(track_pitch_um / obstacle_map.grid.grid_size_um))
    assignments_by_terminal_id = {
        assignment.terminal.id: assignment
        for assignment in pad_plan.assignments
        if assignment.kind == "individual" and assignment.terminal is not None
    }
    target_cells_by_slot = {
        assignment.slot.index: pad_access_cells(assignment.slot, obstacle_map, config)
        for assignment in pad_plan.assignments
    }
    topology_route_by_terminal_id = {
        route.terminal.id: route for route in topology.routes if route.success
    }
    all_terminal_cells = (
        set().union(*_individual_terminal_open_cells(obstacle_map).values())
        if _individual_terminal_open_cells(obstacle_map)
        else set()
    )
    hard_blocked = set(obstacle_map.blocked_cells)
    hard_blocked.update(common_bus.tree_cells)
    hard_blocked.update(common_bus_escape.path)
    # The escape is realized at config.bus_width_um (wider than its
    # centerline), so it is blocked here at its realized rectangles -- the
    # same ones metal_realization.realize_electrical_metal draws -- not just
    # the centerline cells above.
    escape_rects = common_bus_escape_rects(common_bus, common_bus_escape, obstacle_map, config)
    escape_metal_cells: set[GridCell] = set().union(
        *(bbox_to_grid_cells(rect, obstacle_map.grid) for rect in escape_rects)
    )
    hard_blocked.update(escape_metal_cells)

    routes: list[DetailedBundleRoute] = []
    failed_routes: list[DetailedBundleRoute] = []
    cell_usage: dict[GridCell, int] = {}
    committed_cells: set[GridCell] = set()
    committed_footprint_cells: set[GridCell] = set()
    assigned_pad_cells_by_slot = {
        assignment.slot.index: bbox_to_grid_cells(assignment.slot.bbox, obstacle_map.grid)
        for assignment in pad_plan.assignments
    }
    all_assigned_pad_cells = (
        set().union(*assigned_pad_cells_by_slot.values()) if assigned_pad_cells_by_slot else set()
    )

    def commit(
        detailed_path: tuple[tuple[float, float], ...],
        *,
        bundle: EscapeBundle,
        rank: int,
        terminal: ElectricalTerminal,
        assignment: PadAssignment,
        topology_route: EscapeTopologyRoute,
        target_cells: frozenset[GridCell],
        shape: Literal["L", "Z"],
        contact_port_name: str | None,
        construction: Literal["river", "fallback"],
    ) -> None:
        path = _cells_from_point_path(detailed_path)
        route = DetailedBundleRoute(
            bundle_id=bundle.bundle_id,
            rank=rank,
            terminal=terminal,
            pad_assignment=assignment,
            path=path,
            target_cells=frozenset(target_cells),
            track_cell=_representative_track_cell(topology_route),
            lane_cell=_representative_lane_cell(path, target_cells, config),
            offset_um=0.0,
            offset_axis=_offset_axis(bundle),
            offset_path=detailed_path,
            source_stub_path=detailed_path,
            bundle_track_path=(),
            pad_stub_path=detailed_path,
            access_anchor_cell=topology_route.access_anchor_cell,
            route_start_cell=topology_route.route_start_cell,
            used_access_anchor=topology_route.used_access_anchor,
            shape=shape,
            contact_port_name=contact_port_name,
            construction=construction,
            success=True,
        )
        routes.append(route)
        committed_cells.update(path)
        committed_footprint_cells.update(
            _wire_reservation_cells_from_point_path(
                detailed_path,
                obstacle_map,
                config,
                radius=_wire_spacing_radius_cells(obstacle_map, config),
            )
        )
        for cell in path:
            cell_usage[cell] = cell_usage.get(cell, 0) + 1

    def fail(
        *,
        bundle_id: int,
        rank: int,
        terminal: ElectricalTerminal,
        assignment: PadAssignment | None,
        target_cells: frozenset[GridCell],
        reason: str,
    ) -> None:
        failed_routes.append(
            DetailedBundleRoute(
                bundle_id=bundle_id,
                rank=rank,
                terminal=terminal,
                pad_assignment=assignment,
                path=(),
                target_cells=frozenset(target_cells),
                track_cell=None,
                lane_cell=None,
                offset_um=0.0,
                offset_axis="x",
                offset_path=(),
                success=False,
                reason=reason,
            )
        )

    # Bundles in pad-row order (``topology.bundles`` is already sorted by
    # exit position along the pad row). Each bundle is first tried as a
    # whole with river routing (nested corridor/channel lanes, Milestone 2);
    # if any wire's segment collides, the whole bundle falls back to the
    # per-wire full-grid A* instead, nesting order, committing one by one so
    # a later wire in the same bundle sees the earlier ones' footprints.
    bundle_construction_notes: list[tuple[int, str, str]] = []
    for bundle in topology.bundles:
        route_side = bundle_route_side(bundle, obstacle_map)
        exit_dx = 1 if route_side == "right" else -1
        members: list[
            tuple[int, ElectricalTerminal, PadAssignment, EscapeTopologyRoute, frozenset[GridCell]]
        ] = []
        for rank, terminal in tuple(reversed(_route_order_for_bundle(bundle, route_side))):
            assignment = assignments_by_terminal_id.get(terminal.id)
            topology_route = topology_route_by_terminal_id.get(terminal.id)
            target_cells = (
                target_cells_by_slot.get(assignment.slot.index, frozenset())
                if assignment is not None
                else frozenset()
            )
            failure_reason = _detail_failure_reason(
                terminal,
                assignment,
                topology_route,
                target_cells,
                obstacle_map,
                hard_blocked,
                all_terminal_cells,
            )
            if failure_reason is not None:
                fail(
                    bundle_id=bundle.bundle_id,
                    rank=rank,
                    terminal=terminal,
                    assignment=assignment,
                    target_cells=target_cells,
                    reason=failure_reason,
                )
                continue
            assert assignment is not None
            assert topology_route is not None
            members.append((rank, terminal, assignment, topology_route, target_cells))

        if not members:
            continue

        river_wires, river_failure_reason = _try_river_route_bundle(
            members,
            exit_dx,
            obstacle_map,
            config,
            hard_blocked,
            all_terminal_cells,
            all_assigned_pad_cells,
            assigned_pad_cells_by_slot,
            committed_footprint_cells,
        )
        if river_wires is not None:
            for (
                rank,
                terminal,
                assignment,
                topology_route,
                target_cells,
            ), centerline, shape in river_wires:
                terminal_access = obstacle_map.individual_port_accesses[terminal.id]
                commit(
                    centerline,
                    bundle=bundle,
                    rank=rank,
                    terminal=terminal,
                    assignment=assignment,
                    topology_route=topology_route,
                    target_cells=target_cells,
                    shape=shape,
                    contact_port_name=_pad_wire_contact_port_name(
                        terminal, terminal_access, centerline
                    ),
                    construction="river",
                )
            continue

        bundle_construction_notes.append(
            (bundle.bundle_id, "fallback", river_failure_reason or "unknown reason")
        )
        for rank, terminal, assignment, topology_route, target_cells in members:
            terminal_access = obstacle_map.individual_port_accesses.get(terminal.id)
            source_cells = _terminal_open_cells(obstacle_map, terminal.id)
            own_pad_cells = assigned_pad_cells_by_slot.get(assignment.slot.index, target_cells)
            raw_fallback_blocked = set(hard_blocked)
            raw_fallback_blocked.update(all_terminal_cells.difference(source_cells))
            raw_fallback_blocked.update(all_assigned_pad_cells.difference(own_pad_cells))
            detailed_path = (
                _route_full_grid_pad_wire(
                    terminal_access,
                    target_cells,
                    obstacle_map,
                    raw_fallback_blocked,
                    committed_footprint_cells,
                    config,
                )
                if terminal_access is not None
                else ()
            )
            if not detailed_path:
                fail(
                    bundle_id=bundle.bundle_id,
                    rank=rank,
                    terminal=terminal,
                    assignment=assignment,
                    target_cells=target_cells,
                    reason="Z fallback cannot reach its pad without crossing committed metal "
                    "or obstacles",
                )
                continue
            # A wire that would short must fail, never be drawn: the fallback
            # search already avoids committed_footprint_cells, so this should
            # never trip, but it is the same safety net as Milestone 2 added
            # and is kept as a belt-and-suspenders check. committed_footprint_
            # cells already carries the spacing-radius dilation (see commit()
            # above), so it is tested against the fallback path's bare
            # centerline, not a re-dilated one -- dilating twice would hold
            # two wires apart by twice their required clearance.
            fallback_bare_cells = set(_cells_from_point_path(detailed_path))
            if fallback_bare_cells & committed_footprint_cells:
                fail(
                    bundle_id=bundle.bundle_id,
                    rank=rank,
                    terminal=terminal,
                    assignment=assignment,
                    target_cells=target_cells,
                    reason="Z fallback route crosses a sibling's committed metal footprint",
                )
                continue
            assert terminal_access is not None
            fallback_bends = _count_direction_changes(detailed_path)
            commit(
                detailed_path,
                bundle=bundle,
                rank=rank,
                terminal=terminal,
                assignment=assignment,
                topology_route=topology_route,
                target_cells=target_cells,
                shape="L" if fallback_bends <= 1 else "Z",
                contact_port_name=_pad_wire_contact_port_name(
                    terminal, terminal_access, detailed_path
                ),
                construction="fallback",
            )

    routed_terminal_ids = {route.terminal.id for route in routes}
    routed_terminal_ids.update(route.terminal.id for route in failed_routes)
    for assignment in sorted(
        (
            assignment
            for assignment in pad_plan.assignments
            if assignment.kind == "individual" and assignment.terminal is not None
        ),
        key=lambda assignment: assignment.slot.index,
    ):
        terminal = assignment.terminal
        if terminal is None or terminal.id in routed_terminal_ids:
            continue
        failed_routes.append(
            DetailedBundleRoute(
                bundle_id=assignment.topology_bundle_id
                if assignment.topology_bundle_id is not None
                else -1,
                rank=assignment.topology_rank if assignment.topology_rank is not None else -1,
                terminal=terminal,
                pad_assignment=assignment,
                path=(),
                target_cells=frozenset(
                    target_cells_by_slot.get(assignment.slot.index, frozenset())
                ),
                track_cell=None,
                lane_cell=None,
                offset_um=0.0,
                offset_axis="x",
                offset_path=(),
                success=False,
                reason="terminal was not present in any escape topology bundle",
            )
        )

    return DetailedBundleRoutingResult(
        routes=tuple(sorted(routes, key=_detailed_route_sort_key)),
        failed_routes=tuple(sorted(failed_routes, key=_detailed_route_sort_key)),
        committed_cells=frozenset(committed_cells),
        cell_usage=cell_usage,
        track_pitch_cells=track_pitch_cells,
        bundle_fallback_reasons={
            bundle_id: reason for bundle_id, _kind, reason in bundle_construction_notes
        },
    )


def _route_order_for_bundle(
    bundle: EscapeBundle,
    route_side: RouteSide,
) -> tuple[tuple[int, ElectricalTerminal], ...]:
    ranked_terminals: tuple[tuple[int, ElectricalTerminal], ...] = tuple(
        (rank, terminal) for rank, terminal in enumerate(bundle.ordered_terminals)
    )
    if route_side == "right":
        return tuple(reversed(ranked_terminals))
    return ranked_terminals


def _detailed_route_sort_key(
    route: DetailedBundleRoute,
) -> tuple[int, int, str]:
    return (route.bundle_id, route.rank, route.terminal.id)


# Unused since Milestone 2 (the shelf ranks it computed fed
# _individual_pad_lane_point, itself unused); removed in Milestone 4.
def _pad_lane_rank_maps(
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> tuple[dict[int, int], dict[int, int]]:
    individual_assignments = tuple(
        assignment for assignment in pad_plan.assignments if assignment.kind == "individual"
    )
    if not individual_assignments:
        return {}, {}
    if config.pad_origin_x_um is not None:
        rank_by_slot = {
            assignment.slot.index: rank
            for rank, assignment in enumerate(
                sorted(individual_assignments, key=lambda assignment: assignment.slot.index)
            )
        }
        count_by_slot = {
            assignment.slot.index: len(individual_assignments)
            for assignment in individual_assignments
        }
        return rank_by_slot, count_by_slot

    grouped_assignments: dict[tuple[str, int], list[PadAssignment]] = {}
    fallback_group_id = 0
    for assignment in individual_assignments:
        if assignment.topology_bundle_id is None:
            group_key = ("fallback", fallback_group_id)
            fallback_group_id += 1
        else:
            group_key = ("bundle", assignment.topology_bundle_id)
        grouped_assignments.setdefault(group_key, []).append(assignment)

    rank_by_slot: dict[int, int] = {}
    count_by_slot: dict[int, int] = {}
    for assignments in grouped_assignments.values():
        ordered = sorted(
            assignments,
            key=lambda assignment: (
                assignment.topology_rank
                if assignment.topology_rank is not None
                else assignment.slot.index,
                assignment.slot.index,
            ),
        )
        group_count = len(ordered)
        for rank, assignment in enumerate(ordered):
            rank_by_slot[assignment.slot.index] = rank
            count_by_slot[assignment.slot.index] = group_count
    return rank_by_slot, count_by_slot


def _detail_failure_reason(
    terminal: ElectricalTerminal,
    assignment: PadAssignment | None,
    topology_route: EscapeTopologyRoute | None,
    target_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    hard_blocked: set[GridCell],
    all_terminal_cells: set[GridCell],
) -> str | None:
    if assignment is None:
        return "terminal has no pad assignment"
    if topology_route is None or not topology_route.path:
        return "terminal has no successful topology route"
    if not target_cells:
        return "assigned pad slot has no access cells"
    source_cells = _terminal_open_cells(obstacle_map, terminal.id)
    if not source_cells:
        return "terminal has no source cells in electrical grid"
    route_cells = set(topology_route.path)
    other_terminal_hits = route_cells.intersection(all_terminal_cells.difference(source_cells))
    if other_terminal_hits:
        return "topology route intersects other terminal openings"
    blocked_hits = route_cells.intersection(hard_blocked)
    if blocked_hits:
        return "topology route intersects hard blockers"
    access_anchor = topology_route.access_anchor_cell
    if topology_route.path[0] not in source_cells and topology_route.path[0] != access_anchor:
        return "topology route does not start in source terminal access"
    return None


def _target_cell(target_cells: frozenset[GridCell], config: ElectricalRoutingConfig) -> GridCell:
    if config.pad_side == "top":
        edge_y = min(y for _, y in target_cells)
    else:
        edge_y = max(y for _, y in target_cells)
    center_x = round((min(x for x, _ in target_cells) + max(x for x, _ in target_cells)) / 2.0)
    return min(
        target_cells, key=lambda cell: (abs(cell[1] - edge_y), abs(cell[0] - center_x), cell)
    )


def _cell_center(cell: GridCell) -> tuple[float, float]:
    return (cell[0] + 0.5, cell[1] + 0.5)


def _try_river_route_bundle(
    members: list[
        tuple[int, ElectricalTerminal, PadAssignment, EscapeTopologyRoute, frozenset[GridCell]]
    ],
    exit_dx: int,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    hard_blocked: set[GridCell],
    all_terminal_cells: set[GridCell],
    all_assigned_pad_cells: set[GridCell],
    assigned_pad_cells_by_slot: dict[int, frozenset[GridCell]],
    committed_footprint_cells: set[GridCell],
) -> tuple[
    list[
        tuple[
            tuple[int, ElectricalTerminal, PadAssignment, EscapeTopologyRoute, frozenset[GridCell]],
            tuple[GridPoint, ...],
            Literal["L", "Z"],
        ]
    ]
    | None,
    str | None,
]:
    """River-route a whole bundle: nested corridor and channel lanes.

    2026-09-25 (electrical routing quality plan, Milestone 2): a bundle of
    k terminals stacked on one exit side is routed together, not wire by
    wire. Row i = 1..k, numbered from the terminal nearest the pad row
    outward (the same nesting order pad assignment already uses): wire i
    runs from its port at its own terminal row out to corridor lane
    c_i = exit_column + i * track_pitch (exit_column just past every
    member's own terminal opening, plus clearance, on the exit side), up
    that lane to channel lane h_i = pad_access_row - i * track_pitch (the
    same shelf arithmetic ``_individual_pad_lane_point`` used, applied
    directly by row instead of through its lane-rank/side indirection),
    across the channel to its own pad column, then into the pad. c_i
    increases and h_i decreases with i, so wire i's channel run passes
    above every wire j > i's corridor run and left of its pad run: no two
    wires cross (by construction; asserted separately over the built
    paths). Every run is monotone in x and y, so total length always equals
    the Manhattan distance (detour 0); the wire collapses to a single L
    when its corridor lane already is its pad column.

    Returns ``(None, reason)`` if any member has no port access, if a corridor
    or channel lane ends up off the grid, if a pad column ends up on the
    wrong side of a corridor lane (breaking monotonicity), or if a wire
    collides -- checked in two passes: the wire's centerline dilated by the
    obstacle-clearance radius against the obstacle map, every other
    terminal's opening and every other pad; then the wire's bare centerline
    against ``committed_footprint_cells`` (every already-committed wire's
    centerline, already dilated by the spacing radius when it was
    committed). The caller falls the whole bundle back to the per-wire
    full-grid search in that case. Otherwise returns the built wires with
    the reason left ``None``.
    """

    if exit_dx == 0:
        return None, "bundle exit direction is unknown"

    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    track_pitch_cells = max(1, math.ceil(track_pitch_um / obstacle_map.grid.grid_size_um))
    clearance_cells = max(
        1, math.ceil(max(config.obstacle_clearance_um, 0.0) / obstacle_map.grid.grid_size_um)
    )

    terminal_accesses: list[ElectricalPortAccess] = []
    for _rank, terminal, _assignment, _topology_route, _target_cells in members:
        access = obstacle_map.individual_port_accesses.get(terminal.id)
        if access is None:
            return None, f"{terminal.id} has no individual port access"
        terminal_accesses.append(access)

    open_xs = [
        x
        for _rank, terminal, _assignment, _topology_route, _target_cells in members
        for x, _y in _terminal_open_cells(obstacle_map, terminal.id)
    ]
    if not open_xs:
        open_xs = [access.anchor_cell[0] for access in terminal_accesses]
    opening_edge = max(open_xs) if exit_dx > 0 else min(open_xs)
    exit_column = opening_edge + exit_dx * clearance_cells
    obstacle_radius = _wire_reservation_radius_cells(obstacle_map, config)

    wires: list[
        tuple[
            tuple[int, ElectricalTerminal, PadAssignment, EscapeTopologyRoute, frozenset[GridCell]],
            tuple[GridPoint, ...],
            Literal["L", "Z"],
        ]
    ] = []
    for row_index, (member, access) in enumerate(zip(members, terminal_accesses, strict=True), 1):
        _rank, terminal, assignment, _topology_route, target_cells = member
        if not target_cells:
            return None, f"{terminal.id} has no pad access cells"

        anchor_cell = access.anchor_cell
        c_i = exit_column + exit_dx * row_index * track_pitch_cells
        if not (0 <= c_i < obstacle_map.grid.width):
            return None, f"{terminal.id}'s corridor lane is off the grid"
        pad_cell = _target_cell(target_cells, config)
        p_i = pad_cell[0]
        if exit_dx * (p_i - c_i) < 0:
            return None, f"{terminal.id}'s pad column is inward of its corridor lane"
        h_i = _river_channel_lane_y(target_cells, config, row_index, track_pitch_cells)
        if not (0 <= h_i < obstacle_map.grid.height):
            return None, f"{terminal.id}'s channel lane is off the grid"
        if config.pad_side == "top" and h_i < anchor_cell[1]:
            return None, f"{terminal.id}'s channel lane is below its own terminal row"
        if config.pad_side == "bottom" and h_i > anchor_cell[1]:
            return None, f"{terminal.id}'s channel lane is above its own terminal row"

        centerline = _dedupe_points(
            (
                _cell_center(anchor_cell),
                _cell_center((c_i, anchor_cell[1])),
                _cell_center((c_i, h_i)),
                _cell_center((p_i, h_i)),
                _cell_center(pad_cell),
            )
        )
        if len(centerline) < 2:
            return None, f"{terminal.id}'s river centerline is degenerate"

        source_cells = _terminal_open_cells(obstacle_map, terminal.id)
        own_pad_cells = assigned_pad_cells_by_slot.get(assignment.slot.index, target_cells)
        member_hard_blocked = set(hard_blocked)
        member_hard_blocked.update(all_terminal_cells.difference(source_cells))
        member_hard_blocked.update(all_assigned_pad_cells.difference(own_pad_cells))
        reservation = _wire_reservation_cells_from_point_path(
            centerline, obstacle_map, config, radius=obstacle_radius
        )
        if reservation & member_hard_blocked:
            return None, f"{terminal.id}'s river wire collides at row {row_index}"
        # Checked separately at the bare centerline, not dilated again:
        # committed_footprint_cells already carries the spacing-radius
        # dilation from commit(), so re-dilating here would hold two wires
        # apart by twice their required clearance.
        if set(_cells_from_point_path(centerline)) & committed_footprint_cells:
            return (
                None,
                f"{terminal.id}'s river wire is too close to a committed wire at row {row_index}",
            )

        bends = _count_direction_changes(centerline)
        shape: Literal["L", "Z"] = "L" if bends <= 1 else "Z"
        wires.append((member, centerline, shape))
        # Not folded into member_hard_blocked or committed_footprint_cells:
        # c_i increasing and h_i decreasing with row_index already keeps
        # every wire clear of every other wire in this same bundle (the
        # crossing test on the construction checks this); the obstacle
        # radius here is sized for the obstacle clearance, not the (usually
        # smaller) track pitch two parallel lanes of the same bundle are
        # deliberately spaced by, so checking a wire against its own
        # bundle's other rows would produce false collisions at the pitch
        # boundary.

    return wires, None


def _river_channel_lane_y(
    target_cells: frozenset[GridCell],
    config: ElectricalRoutingConfig,
    row_index: int,
    track_pitch_cells: int,
) -> int:
    """Channel lane height h_i = pad access row -+ row_index * track pitch.

    Same shelf arithmetic as ``_individual_pad_lane_point``'s
    ``target_edge_y +- shelf_index * track_pitch_cells``, applied directly
    by row index (1 = nearest the pad row) since the river construction
    already knows row order unambiguously, instead of going through that
    function's lane-rank/lane-count/direction_x indirection (built for a
    different caller that did not know row order in advance).

    Not clamped to the grid: the caller (``_try_river_route_bundle``) refuses
    a lane that ends up off the grid instead of silently sharing a lane with
    another row at the edge.
    """

    if config.pad_side == "top":
        target_edge_y = min(y for _, y in target_cells)
        return target_edge_y - row_index * track_pitch_cells
    target_edge_y = max(y for _, y in target_cells)
    return target_edge_y + row_index * track_pitch_cells


def _count_direction_changes(points: tuple[GridPoint, ...]) -> int:
    """Return the number of direction changes along a Manhattan centerline."""

    if len(points) < 3:
        return 0
    count = 0
    previous_direction = _point_direction(points[0], points[1])
    # Intentional: points[1:] and points[2:] differ in length by one, a
    # sliding window over consecutive direction segments, not a bug.
    for start, end in zip(points[1:], points[2:], strict=False):
        current_direction = _point_direction(start, end)
        if current_direction != (0, 0) and previous_direction != (0, 0):
            if current_direction != previous_direction:
                count += 1
            previous_direction = current_direction
        elif current_direction != (0, 0):
            previous_direction = current_direction
    return count


def _terminal_exit_direction_x(terminal_access: ElectricalPortAccess) -> int:
    """Return -1/+1 for a terminal's ``:l``/``:r`` exit side, 0 if unknown."""

    side_key = terminal_access.terminal_id.rsplit(":", 1)[-1]
    if side_key == "l":
        return -1
    if side_key == "r":
        return 1
    return 0


def _exit_facing_port_name(
    terminal: ElectricalTerminal,
    exit_dx: int,
) -> str | None:
    """Return the terminal's physical port that faces an L wire's exit side.

    The port used to anchor the escape route to the routing grid (chosen to
    face the pad side, i.e. vertically) is not necessarily the port whose
    physical orientation matches the L's horizontal first segment. Landing
    the metal contact on the port that does face that direction keeps the
    adapter a straight join instead of an up-then-back-down elbow.
    """

    if not terminal.ports or exit_dx == 0:
        return None
    target_orientation = 180.0 if exit_dx < 0 else 0.0
    return min(
        terminal.ports,
        key=lambda port: (
            _orientation_distance(port.orientation, target_orientation),
            port.name,
        ),
    ).name


def _pad_wire_contact_port_name(
    terminal: ElectricalTerminal,
    terminal_access: ElectricalPortAccess,
    centerline: tuple[GridPoint, ...],
) -> str | None:
    """Return the contact port name for a pad wire's metal adapter.

    Shared by the L construction and the full-grid Z fallback: either can
    start with a real horizontal run (an L's own run, or the fallback's
    forced first step in the terminal's exit direction), in which case the
    wire lands on the terminal's horizontally facing port (an l_e1/r_e3-style
    port) rather than the vertically facing port used to anchor the escape
    route to the grid, so the adapter is a straight join. A wire with no such
    run (a degenerate, purely vertical L) keeps the grid-anchor port.
    """

    if (
        len(centerline) >= 2
        and centerline[0][1] == centerline[1][1]
        and centerline[0][0] != centerline[1][0]
    ):
        return _exit_facing_port_name(terminal, _terminal_exit_direction_x(terminal_access))
    return terminal_access.port_name


def _route_full_grid_pad_wire(
    terminal_access: ElectricalPortAccess,
    pad_target_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    raw_blocked_cells: set[GridCell],
    committed_footprint_cells: set[GridCell],
    config: ElectricalRoutingConfig,
) -> tuple[GridPoint, ...]:
    """Full-grid A* fallback from a terminal's port anchor to its pad.

    Used only for a wire ``_try_river_route_bundle`` refused (the pad column
    is on the wrong side of the exit direction, or a run is blocked).
    Searches the whole electrical grid, avoiding the obstacle map, every
    other terminal's opening, and every other pad (``raw_blocked_cells``,
    dilated here by the obstacle-clearance radius, matching the clearance a
    river wire requires by dilating its own centerline instead), plus every
    already-committed wire's reservation footprint
    (``committed_footprint_cells``, already dilated by the spacing radius --
    wire width plus the wider of the obstacle clearance and the individual
    route spacing -- when it was committed; not dilated again, or two wires
    would be held twice their required clearance apart). The first step is
    forced into the terminal's own exit direction so the search cannot double
    back through the terminal's own opening from the wrong side.

    Before searching, ``_pad_reachable`` runs a plain breadth-first search to
    rule out an unreachable pad cheaply: the A* below would otherwise exhaust
    every direction state of the grid (width * height * 4) before concluding
    a pad cannot be reached at all.
    """

    exit_dx = _terminal_exit_direction_x(terminal_access)
    forced_first_direction = (exit_dx, 0) if exit_dx != 0 else None
    obstacle_radius = _wire_reservation_radius_cells(obstacle_map, config)
    blocked = _dilate_cells(
        raw_blocked_cells, obstacle_radius, obstacle_map.grid.width, obstacle_map.grid.height
    )
    blocked.update(committed_footprint_cells)
    if not _pad_reachable(
        terminal_access.anchor_cell,
        pad_target_cells,
        blocked,
        obstacle_map.grid.width,
        obstacle_map.grid.height,
        forced_first_direction,
    ):
        return ()
    return _route_pad_stub_path(
        start=_cell_center(terminal_access.anchor_cell),
        target_cells=pad_target_cells,
        blocked=blocked,
        obstacle_map=obstacle_map,
        config=config,
        forced_first_direction=forced_first_direction,
    )


def _pad_reachable(
    start_cell: GridCell,
    targets: frozenset[GridCell],
    blocked: set[GridCell],
    width: int,
    height: int,
    forced_first_direction: tuple[int, int] | None,
) -> bool:
    """Plain 4-connected BFS: can ``start_cell`` reach any of ``targets``?

    Mirrors ``_route_pad_stub_path``'s blocked handling (the targets and the
    start cell are never treated as blocking) and its forced first move, but
    with no direction states and no costs, so it settles an unreachable pad
    in one bounded pass over undirected cells instead of the full A* state
    space.
    """

    targets = frozenset(cell for cell in targets if _in_bounds(cell, width, height))
    if not targets:
        return False
    if start_cell in targets:
        return True

    local_blocked = set(blocked)
    local_blocked.difference_update(targets)
    local_blocked.discard(start_cell)

    if forced_first_direction is not None:
        first_cell = (
            start_cell[0] + forced_first_direction[0],
            start_cell[1] + forced_first_direction[1],
        )
        if not _in_bounds(first_cell, width, height) or first_cell in local_blocked:
            return False
        frontier_start = first_cell
    else:
        frontier_start = start_cell

    if frontier_start in targets:
        return True
    visited = {start_cell, frontier_start}
    queue: deque[GridCell] = deque((frontier_start,))
    while queue:
        cell = queue.popleft()
        for dx, dy in _GRID_DIRECTIONS:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if neighbor in visited or not _in_bounds(neighbor, width, height):
                continue
            if neighbor in local_blocked:
                continue
            visited.add(neighbor)
            if neighbor in targets:
                return True
            queue.append(neighbor)
    return False


def _dilate_cells(
    cells: set[GridCell],
    radius: int,
    width: int,
    height: int,
) -> set[GridCell]:
    """Return ``cells`` expanded by ``radius`` in every direction (Chebyshev)."""

    if radius <= 0:
        return set(cells)
    dilated: set[GridCell] = set()
    for x, y in cells:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (x + dx, y + dy)
                if _in_bounds(cell, width, height):
                    dilated.add(cell)
    return dilated


# Unused since Milestone 2 (L-shaped pad wires with a full-grid A* fallback
# replaced the lane/shelf pad-wire construction below); removed in
# Milestone 4. Kept for now so the diff stays reviewable.
def _individual_pad_lane_point(
    target_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    track_end: tuple[float, float],
    pad_lane_rank: int,
    pad_lane_count: int,
) -> tuple[float, float]:
    target = _target_cell(target_cells, config)
    direction_x = 1 if target[0] + 0.5 >= track_end[0] else -1
    track_pitch_cells = max(
        1,
        math.ceil(
            (config.wire_width_um + config.individual_route_spacing_um)
            / obstacle_map.grid.grid_size_um
        ),
    )
    if direction_x >= 0:
        shelf_index = pad_lane_rank + 1
    else:
        shelf_index = pad_lane_count - pad_lane_rank
    if config.pad_side == "top":
        target_edge_y = min(y for _, y in target_cells)
        lane_y = target_edge_y - shelf_index * track_pitch_cells
    else:
        target_edge_y = max(y for _, y in target_cells)
        lane_y = target_edge_y + shelf_index * track_pitch_cells
    lane_y = min(max(0, lane_y), obstacle_map.grid.height - 1)
    return (target[0] + 0.5, lane_y + 0.5)


# Unused since Milestone 2 (the lane offset it stitched together for pad
# wires is now either a clean L or the full-grid A* fallback); removed in
# Milestone 4.
def _route_prefix_to_pad_stub_start(
    *,
    source_point: tuple[float, float],
    bundle_track_path: tuple[tuple[float, float], ...],
    pad_lane: tuple[float, float],
    pad_side: str,
) -> tuple[
    tuple[tuple[float, float], ...],
    tuple[tuple[float, float], ...],
    tuple[tuple[float, float], ...],
    tuple[float, float],
]:
    if not bundle_track_path:
        source_stub = _manhattan_point_path(source_point, pad_lane)
        return source_stub, source_stub, (), source_stub[-1]

    attach_point, track_tail = _attach_point_and_tail(
        source_point,
        bundle_track_path,
        pad_side=pad_side,
    )
    source_stub = _manhattan_point_path(source_point, attach_point)
    track_tail = _truncate_track_tail_at_pad_lane(track_tail, pad_lane)
    pad_stub_start = track_tail[-1] if track_tail else attach_point
    prefix = _dedupe_points((*source_stub, *track_tail[1:]))
    return prefix, source_stub, track_tail, pad_stub_start


def _truncate_track_tail_at_pad_lane(
    track_tail: tuple[tuple[float, float], ...],
    pad_lane: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    if len(track_tail) <= 1:
        return track_tail
    lane_y = pad_lane[1]
    truncated: list[tuple[float, float]] = [track_tail[0]]
    for start, end in zip(track_tail, track_tail[1:]):
        if _segment_crosses_horizontal_line(start, end, lane_y):
            branch = _project_horizontal_line_to_segment(start, end, lane_y)
            if branch != truncated[-1]:
                truncated.append(branch)
            return _dedupe_points(tuple(truncated))
        if end != truncated[-1]:
            truncated.append(end)
    return _dedupe_points(tuple(truncated))


def _segment_crosses_horizontal_line(
    start: tuple[float, float],
    end: tuple[float, float],
    y: float,
) -> bool:
    low_y, high_y = sorted((start[1], end[1]))
    return low_y <= y <= high_y


def _project_horizontal_line_to_segment(
    start: tuple[float, float],
    end: tuple[float, float],
    y: float,
) -> tuple[float, float]:
    if start[0] == end[0]:
        return (start[0], y)
    if start[1] == end[1]:
        low_x, high_x = sorted((start[0], end[0]))
        return (min(max(start[0], low_x), high_x), y)
    return (start[0], y)


# Unused since Milestone 2 (only _route_prefix_to_pad_stub_start called
# this); removed in Milestone 4.
def _attach_point_and_tail(
    source_point: tuple[float, float],
    path: tuple[tuple[float, float], ...],
    *,
    pad_side: str,
) -> tuple[tuple[float, float], tuple[tuple[float, float], ...]]:
    if len(path) == 1:
        return path[0], path

    best_index = 0
    best_projection = path[0]
    best_key: tuple[float, float, int] | None = None
    for index, (start, end) in enumerate(zip(path, path[1:])):
        projection = _project_point_to_axis_segment(source_point, start, end)
        distance = abs(source_point[0] - projection[0]) + abs(source_point[1] - projection[1])
        pad_direction_score = -projection[1] if pad_side == "top" else projection[1]
        key = (distance, pad_direction_score, index)
        if best_key is None or key < best_key:
            best_key = key
            best_index = index
            best_projection = projection

    tail = _dedupe_points((best_projection, *path[best_index + 1 :]))
    return best_projection, tail


def _project_point_to_axis_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    if start[0] == end[0]:
        low_y, high_y = sorted((start[1], end[1]))
        return (start[0], min(max(point[1], low_y), high_y))
    if start[1] == end[1]:
        low_x, high_x = sorted((start[0], end[0]))
        return (min(max(point[0], low_x), high_x), start[1])
    # Offset paths should be rectilinear. Fall back to the segment start if a
    # malformed diagonal slips through so the route stays debuggable.
    return start


def _manhattan_point_path(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    if start == end:
        return (start,)
    if start[0] == end[0] or start[1] == end[1]:
        return (start, end)
    via = (end[0], start[1])
    return _dedupe_points((start, via, end))


def _route_pad_stub_path(
    *,
    start: tuple[float, float],
    target_cells: frozenset[GridCell],
    blocked: set[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    forced_first_direction: tuple[int, int] | None = None,
) -> tuple[tuple[float, float], ...]:
    """Grid A* from ``start`` to the nearest cell in ``target_cells``.

    Used both for the short local pad stub and, via
    ``_route_full_grid_pad_wire``, as the whole-grid Z fallback. When given,
    ``forced_first_direction`` restricts the very first move out of
    ``start`` to that direction (used to forbid a Z fallback from doubling
    back through its own terminal's opening).
    """

    start_cell = _point_to_cell(start)
    targets = frozenset(
        cell
        for cell in target_cells
        if _in_bounds(cell, obstacle_map.grid.width, obstacle_map.grid.height)
    )
    if not targets:
        return ()

    blocked = set(blocked)
    blocked.difference_update(targets)
    blocked.discard(start_cell)

    start_state: SearchState = (start_cell, _NO_DIRECTION)
    parent: dict[SearchState, SearchState | None] = {start_state: None}
    best_cost: dict[SearchState, int] = {start_state: 0}
    heap: list[tuple[int, int, int, SearchState]] = []
    counter = 0
    heappush(
        heap,
        (
            _pad_stub_heuristic(start_cell, targets),
            0,
            counter,
            start_state,
        ),
    )
    max_expansions = max(1, obstacle_map.grid.width * obstacle_map.grid.height * 4)
    expansions = 0

    while heap and expansions < max_expansions:
        _, cost, _, state = heappop(heap)
        if cost != best_cost.get(state):
            continue
        cell, direction = state
        if cell in targets:
            return _centerline_points(_reconstruct_search_path(parent, state))
        expansions += 1
        for next_direction, neighbor in _pad_stub_neighbors(
            cell,
            targets,
            config,
        ):
            if (
                direction == _NO_DIRECTION
                and forced_first_direction is not None
                and _GRID_DIRECTIONS[next_direction] != forced_first_direction
            ):
                continue
            if not _in_bounds(neighbor, obstacle_map.grid.width, obstacle_map.grid.height):
                continue
            if neighbor in blocked:
                continue
            step_cost = _pad_stub_step_cost(
                cell,
                neighbor,
                direction,
                next_direction,
                targets,
                config,
            )
            next_cost = cost + step_cost
            next_state: SearchState = (neighbor, next_direction)
            if next_cost >= best_cost.get(next_state, 1_000_000_000):
                continue
            parent[next_state] = state
            best_cost[next_state] = next_cost
            counter += 1
            heappush(
                heap,
                (
                    next_cost + _pad_stub_heuristic(neighbor, targets),
                    next_cost,
                    counter,
                    next_state,
                ),
            )
    return ()


def _pad_stub_neighbors(
    cell: GridCell,
    targets: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> tuple[tuple[DirectionIndex, GridCell], ...]:
    x, y = cell
    target_x = round((min(tx for tx, _ in targets) + max(tx for tx, _ in targets)) / 2.0)
    preferred_x_step = 1 if target_x >= x else -1
    preferred_y_step = 1 if config.pad_side == "top" else -1
    ordered_directions = (
        (preferred_x_step, 0),
        (0, preferred_y_step),
        (-preferred_x_step, 0),
        (0, -preferred_y_step),
    )
    return tuple(
        (
            _GRID_DIRECTIONS.index(direction),
            (x + direction[0], y + direction[1]),
        )
        for direction in ordered_directions
    )


def _pad_stub_heuristic(
    cell: GridCell,
    targets: frozenset[GridCell],
) -> int:
    return 10 * min(_manhattan(cell, target) for target in targets)


def _pad_stub_step_cost(
    cell: GridCell,
    neighbor: GridCell,
    direction: DirectionIndex,
    next_direction: DirectionIndex,
    targets: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> int:
    cost = 10
    if direction != _NO_DIRECTION and direction != next_direction:
        cost += 10
    if _distance_to_targets(neighbor, targets) > _distance_to_targets(cell, targets):
        cost += 8
    if _moves_away_from_pad_edge(cell, neighbor, targets, config):
        cost += 4
    if _moves_away_from_target_center_x(cell, neighbor, targets):
        cost += 2
    return cost


def _distance_to_targets(cell: GridCell, targets: frozenset[GridCell]) -> int:
    return min(_manhattan(cell, target) for target in targets)


def _moves_away_from_pad_edge(
    cell: GridCell,
    neighbor: GridCell,
    targets: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> bool:
    _, y = cell
    _, next_y = neighbor
    if config.pad_side == "top":
        target_y = min(target_y for _, target_y in targets)
        return next_y < y and y <= target_y
    target_y = max(target_y for _, target_y in targets)
    return next_y > y and y >= target_y


def _moves_away_from_target_center_x(
    cell: GridCell,
    neighbor: GridCell,
    targets: frozenset[GridCell],
) -> bool:
    x, _ = cell
    next_x, _ = neighbor
    target_x = _target_center_x(targets)
    return abs(next_x - target_x) > abs(x - target_x)


def _target_center_x(targets: frozenset[GridCell]) -> int:
    return round((min(x for x, _ in targets) + max(x for x, _ in targets)) / 2.0)


def _reconstruct_search_path(
    parent: dict[SearchState, SearchState | None],
    end: SearchState,
) -> tuple[GridCell, ...]:
    path: list[GridCell] = []
    current: SearchState | None = end
    while current is not None:
        path.append(current[0])
        current = parent[current]
    path.reverse()
    return _dedupe_path(tuple(path))


def _wire_reservation_cells_from_point_path(
    path: tuple[tuple[float, float], ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    radius: int,
) -> frozenset[GridCell]:
    """Return ``path``'s centerline dilated by ``radius`` cells (Chebyshev).

    ``radius`` is always one of the two reservation radii below, passed
    explicitly by the caller: ``_wire_reservation_radius_cells`` (the
    obstacle-clearance radius) when checking against the obstacle map, or
    ``_wire_spacing_radius_cells`` (the wire-to-wire radius) when building a
    footprint other wires must stay clear of.
    """

    centerline = _cells_from_point_path(path)
    cells: set[GridCell] = set()
    for x, y in centerline:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (x + dx, y + dy)
                if _in_bounds(cell, obstacle_map.grid.width, obstacle_map.grid.height):
                    cells.add(cell)
    return frozenset(cells)


def _wire_reservation_radius_cells(
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    """Return the obstacle-clearance reservation radius, in cells.

    Dilating a wire's centerline by this radius and testing it against the
    obstacle map (or another net's opening/pad cells) holds the wire
    ``wire_width_um + obstacle_clearance_um`` away from them.
    """

    required_center_spacing_um = config.wire_width_um + max(
        config.obstacle_clearance_um,
        0.0,
    )
    return max(
        0,
        math.ceil(required_center_spacing_um / obstacle_map.grid.grid_size_um) - 1,
    )


def _wire_spacing_radius_cells(
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    """Return the wire-to-wire reservation radius, in cells.

    Dilating one wire's centerline by this radius and testing another wire's
    bare centerline against it holds the two centerlines
    ``wire_width_um + individual_route_spacing_um`` apart (and at least the
    obstacle clearance too, so a committed wire's footprint never invites an
    obstacle-clearance violation either). Used to build
    ``committed_footprint_cells``, the reservation every later wire -- river
    or fallback -- must stay clear of.
    """

    required_center_spacing_um = config.wire_width_um + max(
        config.obstacle_clearance_um,
        config.individual_route_spacing_um,
        0.0,
    )
    return max(
        0,
        math.ceil(required_center_spacing_um / obstacle_map.grid.grid_size_um) - 1,
    )


def _cells_from_point_path(path: tuple[tuple[float, float], ...]) -> tuple[GridCell, ...]:
    if not path:
        return ()
    cells: list[GridCell] = []
    for start, end in zip(path, path[1:]):
        start_cell = _point_to_cell(start)
        end_cell = _point_to_cell(end)
        segment = grid_segment(start_cell, end_cell)
        if cells:
            cells.extend(segment[1:])
        else:
            cells.extend(segment)
    if not cells:
        cells.append(_point_to_cell(path[0]))
    return _dedupe_path(tuple(cells))


def _point_to_cell(point: tuple[float, float]) -> GridCell:
    return (round(point[0] - 0.5), round(point[1] - 0.5))


def _dedupe_points(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    deduped: list[tuple[float, float]] = []
    for point in points:
        if deduped and deduped[-1] == point:
            continue
        deduped.append(point)
    return tuple(deduped)


def grid_segment(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
    sx, sy = start
    ex, ey = end
    cells: list[GridCell] = []
    if sx != ex:
        step_x = 1 if ex > sx else -1
        cells.extend((x, sy) for x in range(sx, ex + step_x, step_x))
    else:
        cells.append((sx, sy))
    if sy != ey:
        step_y = 1 if ey > sy else -1
        start_y = sy + step_y
        cells.extend((ex, y) for y in range(start_y, ey + step_y, step_y))
    return tuple(cells)


def _in_bounds(cell: GridCell, width: int, height: int) -> bool:
    x, y = cell
    return 0 <= x < width and 0 <= y < height


# Unused since Milestone 2 (the lane offset it computed is no longer used
# for pad wires; DetailedBundleRoute.offset_um is now always 0.0); removed
# in Milestone 4.
def _rank_offset_um(
    rank: int,
    bundle: EscapeBundle,
    obstacle_map: ElectricalObstacleMap,
    track_pitch_um: float,
) -> float:
    if bundle.required_tracks <= 1:
        return 0.0
    if _bundle_route_side(bundle, obstacle_map) == "left":
        return -(bundle.required_tracks - 1 - rank) * track_pitch_um
    return rank * track_pitch_um


# Unused since Milestone 2 (pad wires are an L or a full-grid A* fallback,
# not a lane-offset skeleton); removed in Milestone 4.
def _realize_ordered_bundle_lanes(
    skeleton_path: tuple[GridCell, ...],
    *,
    lane_count: int,
    track_pitch_um: float,
    route_side: str,
    grid_size_um: float,
) -> tuple[tuple[tuple[float, float], ...], ...]:
    """Return lane-preserving bus polylines for a Manhattan skeleton.

    Lane indices are logical, not recomputed from local segment geometry.  The
    initial segment defines the outward side; at each 90-degree bend the local
    side flips so the same physical lane remains ordered through the corner.
    """

    if lane_count <= 0:
        return ()
    points = _centerline_points(skeleton_path)
    if not points:
        return tuple(() for _ in range(lane_count))
    if len(points) == 1:
        return tuple((points[0],) for _ in range(lane_count))

    segments = _manhattan_segments(points)
    if not segments:
        return tuple((points[0],) for _ in range(lane_count))
    track_pitch_cells = track_pitch_um / grid_size_um

    lane_paths: list[tuple[tuple[float, float], ...]] = []
    for lane_index in range(lane_count):
        lane_paths.append(
            _realize_single_ordered_lane(
                segments,
                lane_index=lane_index,
                lane_count=lane_count,
                route_side=route_side,
                track_pitch_cells=track_pitch_cells,
            )
        )
    return tuple(lane_paths)


def _centerline_points(path: tuple[GridCell, ...]) -> tuple[tuple[float, float], ...]:
    simplified = _simplify_manhattan_path(path)
    return tuple(_cell_center(cell) for cell in simplified)


def _manhattan_segments(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in zip(points, points[1:]):
        if start == end:
            continue
        if start[0] != end[0] and start[1] != end[1]:
            raise ValueError("bundle skeleton must be Manhattan")
        if segments:
            previous_direction = _point_direction(*segments[-1])
            current_direction = _point_direction(start, end)
            if (
                previous_direction != current_direction
                and previous_direction != (-current_direction[0], -current_direction[1])
                and previous_direction[0] != 0
                and current_direction[0] != 0
            ):
                raise ValueError("consecutive horizontal bundle segments are invalid")
            if (
                previous_direction != current_direction
                and previous_direction != (-current_direction[0], -current_direction[1])
                and previous_direction[1] != 0
                and current_direction[1] != 0
            ):
                raise ValueError("consecutive vertical bundle segments are invalid")
        segments.append((start, end))
    return tuple(segments)


def _lane_offset_for_direction(
    direction: tuple[int, int],
    lane_index: int,
    *,
    lane_count: int,
    route_side: str,
    track_pitch_cells: float,
) -> tuple[float, float]:
    if lane_count <= 1:
        return (0.0, 0.0)
    dx, dy = direction
    if route_side == "left":
        vertical_x_offset = -(lane_count - 1 - lane_index) * track_pitch_cells
    else:
        vertical_x_offset = lane_index * track_pitch_cells
    if dy != 0:
        return (vertical_x_offset, 0.0)
    if dx < 0:
        return (0.0, vertical_x_offset)
    if dx > 0:
        return (0.0, -vertical_x_offset)
    return (0.0, 0.0)


def _realize_single_ordered_lane(
    segments: tuple[tuple[tuple[float, float], tuple[float, float]], ...],
    *,
    lane_index: int,
    lane_count: int,
    route_side: str,
    track_pitch_cells: float,
) -> tuple[tuple[float, float], ...]:
    segment_points: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in segments:
        offset = _lane_offset_for_direction(
            _point_direction(start, end),
            lane_index,
            lane_count=lane_count,
            route_side=route_side,
            track_pitch_cells=track_pitch_cells,
        )
        segment_points.append((_translate_point(start, offset), _translate_point(end, offset)))

    points: list[tuple[float, float]] = [segment_points[0][0]]
    for index in range(1, len(segment_points)):
        points.append(
            _lane_bend_intersection(
                segment_points[index - 1],
                segment_points[index],
            )
        )
    points.append(segment_points[-1][1])
    return _dedupe_points(tuple(points))


def _translate_point(
    point: tuple[float, float],
    offset: tuple[float, float],
) -> tuple[float, float]:
    return (
        point[0] + offset[0],
        point[1] + offset[1],
    )


def _lane_bend_intersection(
    previous_segment: tuple[tuple[float, float], tuple[float, float]],
    current_segment: tuple[tuple[float, float], tuple[float, float]],
) -> tuple[float, float]:
    previous_start, previous_end = previous_segment
    current_start, current_end = current_segment
    previous_direction = _point_direction(previous_start, previous_end)
    current_direction = _point_direction(current_start, current_end)
    if previous_direction[0] == 0 and current_direction[1] == 0:
        return (previous_end[0], current_start[1])
    if previous_direction[1] == 0 and current_direction[0] == 0:
        return (current_start[0], previous_end[1])
    return previous_end


def _point_direction(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[int, int]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    return (0, 0)


# Unused since Milestone 2 (only _realize_ordered_bundle_lanes called this);
# removed in Milestone 4.
def _bundle_skeleton_path(
    bundle: EscapeBundle,
    config: ElectricalRoutingConfig,
) -> tuple[GridCell, ...]:
    if bundle.shared_cells:
        cells = bundle.shared_cells
    else:
        cells = bundle.cells
    if not cells:
        return ()
    if len(cells) == 1:
        return tuple(cells)
    start = _farthest_cell(min(cells), cells)[0]
    end, parent = _farthest_cell(start, cells)
    path = _reconstruct_cell_path(parent, end)
    if not path:
        return tuple(sorted(cells))
    if config.pad_side == "top" and path[-1][1] < path[0][1]:
        path = tuple(reversed(path))
    elif config.pad_side == "bottom" and path[-1][1] > path[0][1]:
        path = tuple(reversed(path))
    return path


def _farthest_cell(
    start: GridCell,
    cells: frozenset[GridCell],
) -> tuple[GridCell, dict[GridCell, GridCell | None]]:
    queue = [start]
    parent: dict[GridCell, GridCell | None] = {start: None}
    for current in queue:
        for neighbor in _grid_neighbors(current):
            if neighbor not in cells or neighbor in parent:
                continue
            parent[neighbor] = current
            queue.append(neighbor)
    farthest = max(parent, key=lambda cell: (_manhattan(start, cell), cell))
    return farthest, parent


def _reconstruct_cell_path(
    parent: dict[GridCell, GridCell | None],
    end: GridCell,
) -> tuple[GridCell, ...]:
    if end not in parent:
        return ()
    path: list[GridCell] = []
    current: GridCell | None = end
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return tuple(path)


def _grid_neighbors(cell: GridCell) -> tuple[GridCell, GridCell, GridCell, GridCell]:
    x, y = cell
    return ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1))


def _manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _offset_path_by_local_normals(
    path: tuple[GridCell, ...],
    *,
    offset_um: float,
    side: str,
    grid_size_um: float,
) -> tuple[tuple[float, float], ...]:
    if not path:
        return ()
    offset_cells = abs(offset_um) / grid_size_um
    if offset_cells == 0:
        return tuple((x + 0.5, y + 0.5) for x, y in path)

    simplified = _simplify_manhattan_path(path)
    first_cell = next(iter(simplified), None)
    if first_cell is None:
        return ()
    if len(simplified) <= 1:
        x, y = first_cell
        return ((x + 0.5, y + 0.5),)

    normals = [
        _segment_normal(simplified[index], simplified[index + 1], side=side)
        for index in range(len(simplified) - 1)
    ]
    offset_points: list[tuple[float, float]] = []
    for index, cell in enumerate(simplified):
        x, y = cell
        if index == 0:
            normal = normals[0]
            offset_points.append(
                (
                    x + 0.5 + normal[0] * offset_cells,
                    y + 0.5 + normal[1] * offset_cells,
                )
            )
        elif index == len(simplified) - 1:
            normal = normals[-1]
            offset_points.append(
                (
                    x + 0.5 + normal[0] * offset_cells,
                    y + 0.5 + normal[1] * offset_cells,
                )
            )
        else:
            previous = normals[index - 1]
            current = normals[index]
            offset_points.append(
                (
                    x + 0.5 + previous[0] * offset_cells,
                    y + 0.5 + previous[1] * offset_cells,
                )
            )
            offset_points.append(
                (
                    x + 0.5 + current[0] * offset_cells,
                    y + 0.5 + current[1] * offset_cells,
                )
            )
    return _rectilinearize_points(tuple(offset_points))


def _rectilinearize_points(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    if len(points) <= 1:
        return points
    rectilinear: list[tuple[float, float]] = [points[0]]
    for point in points[1:]:
        previous = rectilinear[-1]
        if previous == point:
            continue
        if previous[0] != point[0] and previous[1] != point[1]:
            via = (previous[0], point[1])
            if via != previous and via != point:
                rectilinear.append(via)
        rectilinear.append(point)
    return tuple(rectilinear)


def _simplify_manhattan_path(path: tuple[GridCell, ...]) -> tuple[GridCell, ...]:
    if len(path) <= 2:
        return path
    simplified: list[GridCell] = [path[0]]
    previous_direction = _direction(path[0], path[1])
    for index in range(1, len(path) - 1):
        current_direction = _direction(path[index], path[index + 1])
        if current_direction != previous_direction:
            simplified.append(path[index])
            previous_direction = current_direction
    simplified.append(path[-1])
    return tuple(simplified)


def _direction(start: GridCell, end: GridCell) -> tuple[int, int]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    return (0, 0)


def _segment_normal(start: GridCell, end: GridCell, *, side: str) -> tuple[int, int]:
    dx, dy = _direction(start, end)
    if side == "left":
        return (-dy, dx)
    return (dy, -dx)


def _bundle_route_side(bundle: EscapeBundle, obstacle_map: ElectricalObstacleMap) -> RouteSide:
    return cast(RouteSide, bundle_route_side(bundle, obstacle_map))


def _offset_axis(bundle: EscapeBundle) -> Axis:
    return "x" if bundle.order_axis == "y" else "y"


def _representative_track_cell(route: EscapeTopologyRoute) -> GridCell | None:
    if not route.path:
        return None
    return route.path[min(len(route.path) // 2, len(route.path) - 1)]


def _representative_lane_cell(
    path: tuple[GridCell, ...],
    target_cells: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> GridCell | None:
    if not path or not target_cells:
        return None
    target_y = (
        min(y for _, y in target_cells)
        if config.pad_side == "top"
        else max(y for _, y in target_cells)
    )
    return min(path, key=lambda cell: (abs(cell[1] - target_y), cell))


def _dedupe_path(cells: tuple[GridCell, ...]) -> tuple[GridCell, ...]:
    deduped: list[GridCell] = []
    for cell in cells:
        if deduped and deduped[-1] == cell:
            continue
        deduped.append(cell)
    return tuple(deduped)


def _individual_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    return obstacle_map.individual_terminal_open_cells or obstacle_map.terminal_open_cells


def _terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> frozenset[GridCell]:
    return _individual_terminal_open_cells(obstacle_map).get(
        terminal_id,
        frozenset(),
    )
