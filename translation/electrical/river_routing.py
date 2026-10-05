"""Channel-route every pad wire: one joint track assignment over all bundles.

Every individual pad wire is one horizontal run between two vertical runs: a
corridor lane at x = c (out from the terminal's own row, on its bundle's exit
side) and a pad-access lane at x = p (the assigned pad's column), joined by a
horizontal "track" run at some shared height. A bundle's corridor lanes are
assigned first, in row order (``assign_corridor_lanes``); the track heights
are then assigned once, jointly, over every wire of every bundle
(``assign_tracks``, tracks filled from the pad side down with row
preference), so two bundles whose runs overlap in x never share a height by
accident -- the old per-bundle nesting (and its "inward" mirrored special
case) is a special case of this, not a separate rule. The pad plan
(``pad_slots.plan_pad_slots``) runs the same pure ``assign_tracks`` on the
same lanes and pad columns (``bundle_opening_edge``, ``corridor_lane_column``,
``pad_column_cell``) to size the channel from the global track count.

The crossing argument: a wire W's corridor run rises through every height at
x = c_W, so any other wire R whose horizontal run also crosses x = c_W must
sit above W (nearer the pads) or the two runs cross. Symmetrically, W's own
pad run crosses every height at x = p_W, so any R crossing x = p_W must sit
below W. These two rules, applied to every pair of wires, are the only
vertical constraints the track assignment enforces; a wire whose constraints
form a cycle (no consistent height order exists) cannot be placed and falls
back to the per-wire full-grid search.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
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
    dedupe_points,
    point_direction,
    terminal_open_cells,
    wire_reservation_cells_from_point_path,
    wire_reservation_radius_cells,
)

Member = tuple[int, ElectricalTerminal, PadAssignment, EscapeTopologyRoute, frozenset[GridCell]]
PlacedWire = tuple[Member, tuple[GridPoint, ...], Literal["L", "Z"]]


@dataclass(frozen=True)
class ChannelWire:
    """One wire's track-assignment inputs: an id, its corridor lane and pad column.

    Deliberately independent of every electrical routing type so
    ``assign_tracks`` is pure and unit-testable on synthetic wire sets.
    """

    wire_id: str
    c: int
    p: int
    row: int
    """The wire's row in its bundle, 1 = the terminal nearest the pad row."""

    @property
    def span(self) -> tuple[int, int]:
        return (min(self.c, self.p), max(self.c, self.p))


@dataclass(frozen=True)
class BundleWire:
    """One pad wire's channel-routing geometry, built from its bundle member."""

    member: Member
    anchor_cell: GridCell
    c: int
    p: int
    row: int
    pad_cell: GridCell
    target_cells: frozenset[GridCell]

    @property
    def wire_id(self) -> str:
        return self.member[1].id

    @property
    def row_y(self) -> int:
        return self.anchor_cell[1]

    @property
    def span(self) -> tuple[int, int]:
        return (min(self.c, self.p), max(self.c, self.p))


@dataclass(frozen=True)
class ChannelRoutingResult:
    """The channel router's output: wires ready to commit, and wires to fall back.

    ``placed`` is already in commit order (lowest track first, i.e. farthest
    from the pads first): the caller commits each one before checking the
    next against the committed footprint, exactly as the per-wire fallback
    does. ``unplaced`` carries every wire that never got a track (a whole
    bundle refused at the corridor-lane step, a track-assignment cycle, an
    insufficient-headroom drop, or a per-wire static collision), each with
    its own reason.
    """

    placed: tuple[PlacedWire, ...]
    unplaced: tuple[tuple[Member, str], ...]


def route_bundles_as_channels(
    bundle_entries: list[tuple[int, list[Member]]],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    hard_blocked: set[GridCell],
    all_terminal_cells: set[GridCell],
    all_assigned_pad_cells: set[GridCell],
    assigned_pad_cells_by_slot: dict[int, frozenset[GridCell]],
) -> ChannelRoutingResult:
    """Channel-route every bundle's members jointly: lanes, tracks, paths.

    ``bundle_entries`` is a list of ``(exit_dx, members)``, one per escape
    bundle with at least one member, ``members`` already in row order (row 1
    nearest the pad row -- the caller's existing order). Each bundle's
    corridor lanes are assigned first (``assign_corridor_lanes``); every
    surviving wire from every bundle is then handed to one joint
    ``assign_tracks`` call, so the vertical order between two bundles' runs
    is exactly the crossing argument in the module docstring, never a
    per-bundle guess.
    """

    track_pitch_cells = track_pitch_in_cells(obstacle_map, config)
    obstacle_radius = wire_reservation_radius_cells(obstacle_map, config)

    candidate_wires: list[BundleWire] = []
    unplaced: list[tuple[Member, str]] = []

    for exit_dx, members in bundle_entries:
        if not members:
            continue
        wires, reason = assign_corridor_lanes(members, exit_dx, obstacle_map, config)
        if wires is None:
            for member in members:
                unplaced.append((member, reason or "bundle refused at the corridor-lane step"))
            continue
        candidate_wires.extend(wires)

    if not candidate_wires:
        return ChannelRoutingResult(placed=(), unplaced=tuple(unplaced))

    by_id = {wire.wire_id: wire for wire in candidate_wires}
    channel_wires = [ChannelWire(w.wire_id, w.c, w.p, w.row) for w in candidate_wires]
    track_by_id, cyclic_ids = assign_tracks(channel_wires, track_pitch_cells)
    for wire_id in cyclic_ids:
        unplaced.append((by_id[wire_id].member, "wire is in a track-assignment constraint cycle"))

    pad_edge_y = _pad_access_edge_y(candidate_wires, config)
    track_by_id, dropped = _enforce_track_headroom(
        track_by_id, by_id, config, obstacle_map, track_pitch_cells, pad_edge_y
    )
    for wire_id, reason in dropped:
        unplaced.append((by_id[wire_id].member, reason))

    total_tracks = max(track_by_id.values(), default=0)
    track_y = {
        track: _track_y(track, total_tracks, pad_edge_y, track_pitch_cells, config)
        for track in range(1, total_tracks + 1)
    }

    placed: list[PlacedWire] = []
    for wire_id in sorted(track_by_id, key=lambda wire_id: track_by_id[wire_id]):
        wire = by_id[wire_id]
        centerline = build_wire_path(wire, track_y[track_by_id[wire_id]])
        reason = _check_static_collision(
            wire,
            centerline,
            obstacle_map,
            config,
            hard_blocked,
            all_terminal_cells,
            all_assigned_pad_cells,
            assigned_pad_cells_by_slot,
            obstacle_radius,
        )
        if reason is not None:
            unplaced.append((wire.member, reason))
            continue
        bends = count_direction_changes(centerline)
        shape: Literal["L", "Z"] = "L" if bends <= 1 else "Z"
        placed.append((wire.member, centerline, shape))

    return ChannelRoutingResult(placed=tuple(placed), unplaced=tuple(unplaced))


def assign_corridor_lanes(
    members: list[Member],
    exit_dx: int,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[list[BundleWire] | None, str | None]:
    """Assign one bundle's corridor lanes c_1, c_2, ... in row order.

    Row i = 1..k, numbered from the terminal nearest the pad row outward (the
    order ``members`` is already given in). c_i = exit_column +
    exit_dx * i * track_pitch, exit_column = opening_edge + exit_dx *
    clearance (opening_edge the outermost open cell of any member, on the
    exit side). Refuses the whole bundle only for reasons about its own
    inputs: a member has no port access, no pad-access cells, or its own
    corridor lane lands off the grid. Track heights are not assigned here --
    ``route_bundles_as_channels`` hands every surviving wire to one joint
    ``assign_tracks`` call instead.
    """

    if exit_dx == 0:
        return None, "bundle exit direction is unknown"

    track_pitch_cells = track_pitch_in_cells(obstacle_map, config)
    clearance_cells = clearance_in_cells(obstacle_map, config)

    terminal_accesses: list[ElectricalPortAccess] = []
    for _rank, terminal, _assignment, _topology_route, _target_cells in members:
        access = obstacle_map.individual_port_accesses.get(terminal.id)
        if access is None:
            return None, f"{terminal.id} has no individual port access"
        terminal_accesses.append(access)

    opening_edge = bundle_opening_edge(obstacle_map, [member[1].id for member in members], exit_dx)
    assert opening_edge is not None  # every member has a port access (checked above)

    wires: list[BundleWire] = []
    for row_index, (member, access) in enumerate(zip(members, terminal_accesses, strict=True), 1):
        _rank, terminal, _assignment, _topology_route, target_cells = member
        if not target_cells:
            return None, f"{terminal.id} has no pad access cells"

        anchor_cell = access.anchor_cell
        c_i = corridor_lane_column(
            opening_edge, exit_dx, row_index, track_pitch_cells, clearance_cells
        )
        if not (0 <= c_i < obstacle_map.grid.width):
            return None, f"{terminal.id}'s corridor lane is off the grid"
        pad_cell = pad_column_cell(target_cells, config)

        wires.append(
            BundleWire(
                member=member,
                anchor_cell=anchor_cell,
                c=c_i,
                p=pad_cell[0],
                row=row_index,
                pad_cell=pad_cell,
                target_cells=target_cells,
            )
        )

    return wires, None


def track_pitch_in_cells(
    obstacle_map: ElectricalObstacleMap, config: ElectricalRoutingConfig
) -> int:
    """Return the track pitch (wire width plus individual spacing) in grid cells, at least 1."""

    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    return max(1, math.ceil(track_pitch_um / obstacle_map.grid.grid_size_um))


def clearance_in_cells(obstacle_map: ElectricalObstacleMap, config: ElectricalRoutingConfig) -> int:
    """Return the obstacle clearance in grid cells, at least 1."""

    return max(
        1, math.ceil(max(config.obstacle_clearance_um, 0.0) / obstacle_map.grid.grid_size_um)
    )


def bundle_opening_edge(
    obstacle_map: ElectricalObstacleMap,
    terminal_ids: list[str],
    exit_dx: int,
) -> int | None:
    """Return a bundle's opening edge: its outermost open-cell column on the exit side.

    The column every corridor lane of the bundle is measured from
    (``corridor_lane_column``). Falls back to the terminals' port-access
    anchor columns when no member has an open cell, and to ``None`` when
    no member has a port access either. Shared with the pad plan so that
    its channel sizing sees the same lanes the router will use.
    """

    xs = [
        x
        for terminal_id in terminal_ids
        for x, _y in terminal_open_cells(obstacle_map, terminal_id)
    ]
    if not xs:
        xs = [
            access.anchor_cell[0]
            for terminal_id in terminal_ids
            if (access := obstacle_map.individual_port_accesses.get(terminal_id)) is not None
        ]
    if not xs:
        return None
    return max(xs) if exit_dx > 0 else min(xs)


def corridor_lane_column(
    opening_edge: int,
    exit_dx: int,
    row: int,
    track_pitch_cells: int,
    clearance_cells: int,
) -> int:
    """Return c_row = opening_edge + exit_dx * (clearance + row * track pitch), in cells."""

    return opening_edge + exit_dx * (clearance_cells + row * track_pitch_cells)


def assign_tracks(
    wires: list[ChannelWire],
    track_pitch_cells: int,
) -> tuple[dict[str, int], list[str]]:
    """Assign every wire a track (1 = farthest from the pads): top-down, constrained left-edge.

    Vertical constraints, over every ordered pair of distinct wires (W, R):
    if W's own corridor lane c_W falls inside R's span, R must lie above W
    (R's run would cross W's corridor run); if W's own pad column p_W falls
    inside R's span, W must lie above R (R's run would cross W's pad run).
    "Inside" is widened by one track pitch less one cell on each side of the
    span, not just literal containment: two vertical runs less than a pitch
    apart in x are too close regardless of whether either one's exact x
    falls inside the other's span (measured on cascade_12_folded: a pad
    column two cells short of a lone heater's corridor lane still collides
    once both are dilated by the spacing radius).

    Tracks are filled from the top (the track nearest the pads) down. Each
    round, the wires still unplaced are scanned in row order (row 1, the
    terminal nearest the pad row, first; then by span start) and placed on
    the current track when (a) their run keeps one pitch in x from every
    run already on this track and (b) every wire that must lie above them
    was placed on a strictly earlier, higher track -- checked against the
    placement as it stood before this round, so a same-track dependency
    never counts. A round that places nothing means the remaining wires are
    in a constraint cycle (or blocked by one); they are returned unplaced.
    Filling from the top with row preference keeps every run as near its
    own pad as the constraints allow, so a wire nothing constrains (a lone
    heater beside a nested bundle) no longer sinks to the lowest track at
    the layout's edge, where it met an obstacle on cascade_12_folded
    (2026-09-29). The result is renumbered so that track 1 is the lowest
    track used and the highest is the number of tracks.
    """

    margin = max(0, track_pitch_cells - 1)
    must_be_above: dict[str, set[str]] = {wire.wire_id: set() for wire in wires}
    for wire in wires:
        for other in wires:
            if other.wire_id == wire.wire_id:
                continue
            if _within(wire.c, other.span, margin):
                # other's run crosses wire's corridor lane (or comes within
                # a pitch of it): other above wire.
                must_be_above[wire.wire_id].add(other.wire_id)
            if _within(wire.p, other.span, margin):
                # wire's run crosses other's span at wire's own pad column
                # (or comes within a pitch of it): wire above other.
                must_be_above[other.wire_id].add(wire.wire_id)

    by_id = {wire.wire_id: wire for wire in wires}
    remaining = sorted(
        (wire.wire_id for wire in wires),
        key=lambda wire_id: (by_id[wire_id].row, by_id[wire_id].span[0], wire_id),
    )
    level_by_id: dict[str, int] = {}
    level = 0
    while remaining:
        level += 1
        placed_before_round = set(level_by_id)
        occupied: list[tuple[int, int]] = []
        still_remaining: list[str] = []
        placed_this_round = 0
        for wire_id in remaining:
            wire = by_id[wire_id]
            fits = all(
                wire.span[0] >= end + track_pitch_cells or wire.span[1] + track_pitch_cells <= start
                for start, end in occupied
            )
            deps_ready = must_be_above[wire_id] <= placed_before_round
            if fits and deps_ready:
                level_by_id[wire_id] = level
                occupied.append(wire.span)
                placed_this_round += 1
            else:
                still_remaining.append(wire_id)
        if placed_this_round == 0:
            level -= 1
            break
        remaining = still_remaining

    placed = {wire_id: level + 1 - wire_level for wire_id, wire_level in level_by_id.items()}
    return placed, remaining


def build_wire_path(wire: BundleWire, track_y: int) -> tuple[GridPoint, ...]:
    """Return the four-point (deduplicated) centerline: port -> lane -> track -> pad."""

    return dedupe_points(
        (
            cell_center(wire.anchor_cell),
            cell_center((wire.c, wire.anchor_cell[1])),
            cell_center((wire.c, track_y)),
            cell_center((wire.p, track_y)),
            cell_center(wire.pad_cell),
        )
    )


def _within(value: int, span: tuple[int, int], margin: int = 0) -> bool:
    return span[0] - margin <= value <= span[1] + margin


def _pad_access_edge_y(wires: list[BundleWire], config: ElectricalRoutingConfig) -> int:
    ys = [y for wire in wires for _, y in wire.target_cells]
    return min(ys) if config.pad_side == "top" else max(ys)


def _track_y(
    track: int,
    total_tracks: int,
    pad_edge_y: int,
    track_pitch_cells: int,
    config: ElectricalRoutingConfig,
) -> int:
    offset = (total_tracks + 1 - track) * track_pitch_cells
    if config.pad_side == "top":
        return pad_edge_y - offset
    return pad_edge_y + offset


def _tracks_valid(
    track_by_id: dict[str, int],
    by_id: dict[str, BundleWire],
    config: ElectricalRoutingConfig,
    obstacle_map: ElectricalObstacleMap,
    track_pitch_cells: int,
    pad_edge_y: int,
    clearance_cells: int,
) -> bool:
    if not track_by_id:
        return True
    total_tracks = max(track_by_id.values())
    for track in range(1, total_tracks + 1):
        y = _track_y(track, total_tracks, pad_edge_y, track_pitch_cells, config)
        if not (0 <= y < obstacle_map.grid.height):
            return False
    track1_y = _track_y(1, total_tracks, pad_edge_y, track_pitch_cells, config)
    if config.pad_side == "top":
        boundary_row = max(by_id[wire_id].row_y for wire_id in track_by_id)
        return track1_y >= boundary_row + clearance_cells
    boundary_row = min(by_id[wire_id].row_y for wire_id in track_by_id)
    return track1_y <= boundary_row - clearance_cells


def _enforce_track_headroom(
    track_by_id: dict[str, int],
    by_id: dict[str, BundleWire],
    config: ElectricalRoutingConfig,
    obstacle_map: ElectricalObstacleMap,
    track_pitch_cells: int,
    pad_edge_y: int,
) -> tuple[dict[str, int], list[tuple[str, str]]]:
    """Drop wires, most extreme row first, until the lowest track clears the grid and every row.

    Track 1 (the track farthest from the pads) must still be on the grid and
    clear every placed wire's own terminal row by the obstacle-clearance
    margin; if it does not (too many tracks for the room between the
    terminal rows and the pad row), the wire whose own row is closest to the
    pad edge is dropped to the per-wire fallback and the check repeats. A
    safety net: the pad plan sizes the channel from this same track
    assignment (``pad_slots.plan_pad_slots``), so it triggers only when
    the two disagree (a member the dispatcher refused before routing, or
    a terminal row above the layout's own edge).
    """

    clearance_cells = max(
        1, math.ceil(max(config.obstacle_clearance_um, 0.0) / obstacle_map.grid.grid_size_um)
    )
    track_by_id = dict(track_by_id)
    dropped: list[tuple[str, str]] = []
    while track_by_id and not _tracks_valid(
        track_by_id, by_id, config, obstacle_map, track_pitch_cells, pad_edge_y, clearance_cells
    ):
        if config.pad_side == "top":
            extreme_id = max(track_by_id, key=lambda wire_id: by_id[wire_id].row_y)
        else:
            extreme_id = min(track_by_id, key=lambda wire_id: by_id[wire_id].row_y)
        del track_by_id[extreme_id]
        dropped.append(
            (extreme_id, f"{extreme_id}'s track has insufficient headroom above its own row")
        )
    return track_by_id, dropped


def _check_static_collision(
    wire: BundleWire,
    centerline: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    hard_blocked: set[GridCell],
    all_terminal_cells: set[GridCell],
    all_assigned_pad_cells: set[GridCell],
    assigned_pad_cells_by_slot: dict[int, frozenset[GridCell]],
    obstacle_radius: int,
) -> str | None:
    """Check one wire's dilated centerline against obstacles, other terminals, other pads.

    Does not check ``committed_footprint_cells``: that check is order
    -dependent on commit order, owned by the caller (``bundle_detail_
    router.route_detailed_bundles``), which commits every wire this function
    clears in track order, lowest first, each against the previous ones.
    """

    if len(centerline) < 2:
        return f"{wire.wire_id}'s channel centerline is degenerate"

    terminal_id = wire.wire_id
    _rank, _terminal, assignment, _topology_route, target_cells = wire.member
    source_cells = terminal_open_cells(obstacle_map, terminal_id)
    own_pad_cells = assigned_pad_cells_by_slot.get(assignment.slot.index, target_cells)
    member_hard_blocked = set(hard_blocked)
    member_hard_blocked.update(all_terminal_cells.difference(source_cells))
    member_hard_blocked.update(all_assigned_pad_cells.difference(own_pad_cells))
    reservation = wire_reservation_cells_from_point_path(
        centerline, obstacle_map, config, radius=obstacle_radius
    )
    if reservation & member_hard_blocked:
        return f"{terminal_id}'s channel wire collides"
    return None


def pad_column_cell(target_cells: frozenset[GridCell], config: ElectricalRoutingConfig) -> GridCell:
    """Return the pad-access cell a wire enters its pad through.

    On the chip-facing edge of the pad, at its center column. Shared with
    the pad plan so that its channel sizing sees the same pad columns the
    router will use.
    """

    if config.pad_side == "top":
        edge_y = min(y for _, y in target_cells)
    else:
        edge_y = max(y for _, y in target_cells)
    center_x = round((min(x for x, _ in target_cells) + max(x for x, _ in target_cells)) / 2.0)
    return min(
        target_cells, key=lambda cell: (abs(cell[1] - edge_y), abs(cell[0] - center_x), cell)
    )


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
