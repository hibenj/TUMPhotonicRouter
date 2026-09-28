"""River-route a whole escape bundle as nested corridor/channel lanes."""

from __future__ import annotations

import math
from typing import Literal

from .types import (
    ElectricalObstacleMap,
    ElectricalPortAccess,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    EscapeTopologyRoute,
    GridCell,
    GridPoint,
    PadAssignment,
)
from .wire_geometry import (
    cell_center,
    cells_from_point_path,
    dedupe_points,
    point_direction,
    terminal_open_cells,
    wire_reservation_cells_from_point_path,
    wire_reservation_radius_cells,
)


def try_river_route_bundle(
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
        for x, _y in terminal_open_cells(obstacle_map, terminal.id)
    ]
    if not open_xs:
        open_xs = [access.anchor_cell[0] for access in terminal_accesses]
    opening_edge = max(open_xs) if exit_dx > 0 else min(open_xs)
    exit_column = opening_edge + exit_dx * clearance_cells
    obstacle_radius = wire_reservation_radius_cells(obstacle_map, config)

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

        centerline = dedupe_points(
            (
                cell_center(anchor_cell),
                cell_center((c_i, anchor_cell[1])),
                cell_center((c_i, h_i)),
                cell_center((p_i, h_i)),
                cell_center(pad_cell),
            )
        )
        if len(centerline) < 2:
            return None, f"{terminal.id}'s river centerline is degenerate"

        source_cells = terminal_open_cells(obstacle_map, terminal.id)
        own_pad_cells = assigned_pad_cells_by_slot.get(assignment.slot.index, target_cells)
        member_hard_blocked = set(hard_blocked)
        member_hard_blocked.update(all_terminal_cells.difference(source_cells))
        member_hard_blocked.update(all_assigned_pad_cells.difference(own_pad_cells))
        reservation = wire_reservation_cells_from_point_path(
            centerline, obstacle_map, config, radius=obstacle_radius
        )
        if reservation & member_hard_blocked:
            return None, f"{terminal.id}'s river wire collides at row {row_index}"
        # Checked separately at the bare centerline, not dilated again:
        # committed_footprint_cells already carries the spacing-radius
        # dilation from commit(), so re-dilating here would hold two wires
        # apart by twice their required clearance.
        if set(cells_from_point_path(centerline)) & committed_footprint_cells:
            return (
                None,
                f"{terminal.id}'s river wire is too close to a committed wire at row {row_index}",
            )

        bends = count_direction_changes(centerline)
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


def _target_cell(target_cells: frozenset[GridCell], config: ElectricalRoutingConfig) -> GridCell:
    if config.pad_side == "top":
        edge_y = min(y for _, y in target_cells)
    else:
        edge_y = max(y for _, y in target_cells)
    center_x = round((min(x for x, _ in target_cells) + max(x for x, _ in target_cells)) / 2.0)
    return min(
        target_cells, key=lambda cell: (abs(cell[1] - edge_y), abs(cell[0] - center_x), cell)
    )


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

    Not clamped to the grid: the caller (``try_river_route_bundle``) refuses
    a lane that ends up off the grid instead of silently sharing a lane with
    another row at the edge.
    """

    if config.pad_side == "top":
        target_edge_y = min(y for _, y in target_cells)
        return target_edge_y - row_index * track_pitch_cells
    target_edge_y = max(y for _, y in target_cells)
    return target_edge_y + row_index * track_pitch_cells


def count_direction_changes(points: tuple[GridPoint, ...]) -> int:
    """Return the number of direction changes along a Manhattan centerline."""

    if len(points) < 3:
        return 0
    count = 0
    previous_direction = point_direction(points[0], points[1])
    # Intentional: points[1:] and points[2:] differ in length by one, a
    # sliding window over consecutive direction segments, not a bug.
    for start, end in zip(points[1:], points[2:], strict=False):
        current_direction = point_direction(start, end)
        if current_direction != (0, 0) and previous_direction != (0, 0):
            if current_direction != previous_direction:
                count += 1
            previous_direction = current_direction
        elif current_direction != (0, 0):
            previous_direction = current_direction
    return count
