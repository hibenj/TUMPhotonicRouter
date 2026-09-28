"""River-route a whole escape bundle as nested corridor/channel lanes.

A bundle's pad block is normally outward of its own row-1 corridor lane c_1
("outward" nesting: row i's pad grows outward with i); when ``pad_slots``
can only place it past its own terminal column ("inward" nesting,
``PadAssignment.nesting``), the same corridor lanes c_i are kept but every
wire runs to a mirrored channel height h'_i and then inward to its pad, so
row 1 -- not row k -- reaches the farthest pad. The pad order pad_slots
guarantees (monotonically outward for "outward", monotonically inward for
"inward") keeps every row's channel run clear of every other row's pad run
in the common case, but not always: a row whose own pad straddles far
outward of its own corridor lane can still widen its channel span enough to
engulf an inner row's pad-access run (measured on an eight-row inward
block), so ``try_river_route_bundle`` also checks each new wire's bare
centerline against every earlier row of the same bundle at runtime and
falls the whole bundle back rather than trust the geometry alone.
"""

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

    2026-09-28 (eight-row-bundles plan, Rule A'): row 1's pad column may be
    served even when it is inward of c_1 -- ``pad_slots`` anchors a
    bundle's pad block, in physical space, so row 1's pad lands at or beyond
    c_1, but a later interval's push rule can still shove that block back
    inward by up to one pad pitch (only a "left"-side bundle can be pushed
    this way, since the push only ever increases slot indices). Only row 1
    absorbs this:
    its channel run at h_1 simply reverses direction from c_1 back to p_1
    (the same four-point centerline construction below, now with p_1 on
    the inward side of c_1, so it is a Z rather than an L). It crosses no
    other lane of the bundle because every row j >= 2 has a strictly more
    outward corridor lane c_j (so its vertical run never enters row 1's
    inward excursion) and its own channel height h_j != h_1 (so its
    horizontal run never shares row 1's channel row). Rows >= 2 keep the
    outward-only rule: the pad-slot nesting only ever lets row 1 be pushed
    inward -- *provided* the bundle's own ``PadAssignment.nesting`` is
    still "outward" (below).

    2026-09-28 (mirrored nesting plan): when the whole block ends up on the
    inward side (``nesting == "inward"``, ``pad_slots._interval_nesting``),
    every row uses the same corridor lanes c_i but a mirrored channel rank
    k + 1 - i in place of i (``k`` = the bundle size), so row 1 reaches the
    farthest-inward pad and row k the pad nearest the column -- the order
    ``pad_slots`` already guarantees for an inward block by construction
    (see its docstring), needing no reordering here. The "pad column is
    inward of its corridor lane" refusal below applies only to "outward"
    nesting: for "inward" nesting every row's pad is expected inward of its
    own c_i, and a row whose pad happens to still be outward (a block that
    straddles c_1) is fine too, because the crossing argument only needs
    the pads' relative order, not each row's own side of its own c_i (see
    the module docstring).

    Returns ``(None, reason)`` if any member has no port access, if a corridor
    or channel lane ends up off the grid, if an "outward"-nesting row >= 2
    pad column ends up on the wrong side of a corridor lane (breaking
    monotonicity), or if a wire collides -- checked in three passes: the
    wire's centerline dilated by the obstacle-clearance radius against the
    obstacle map, every other terminal's opening and every other pad; then
    the wire's bare centerline against ``committed_footprint_cells`` (every
    already-committed wire's centerline, already dilated by the spacing
    radius when it was committed); then the wire's bare centerline against
    every earlier row already built in this same bundle (the monotonic pad
    order the module docstring describes does not, on its own, exclude a
    straddling row's channel span engulfing an inner row's pad-access run --
    see the module docstring). The caller falls the whole bundle back to the
    per-wire full-grid search in that case. Otherwise returns the built
    wires with the reason left ``None``.
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

    row_count = len(members)
    nesting: Literal["outward", "inward"] = members[0][2].nesting
    # 2026-09-28 (mirrored nesting plan, gap closed): the module docstring's
    # crossing-freedom argument assumes every row's own pad is on the
    # nesting's expected side of its own corridor lane; a straddling block
    # (explicitly allowed -- some rows outward, e.g. the row farthest from
    # the pad edge in a wide inward block) can break that assumption when
    # that row's own channel span [min(c_i, p_i), max(c_i, p_i)] grows wide
    # enough to engulf an inner row's pad-access run, which the monotonic
    # pad order alone does not exclude (measured: cascade_16_folded's
    # eight-row "extra" group, row 8 straddles far enough outward that its
    # channel run crosses row 7's pad-access run -- confirmed by the
    # verifier as a real ``cross_net_metal_overlap``, not just a shared
    # grid cell). Checked here at the bare centerline, same as the
    # committed-footprint check below, over every row already built in
    # this same bundle; a hit falls the whole bundle back to the per-wire
    # full-grid search, same as any other refusal.
    own_bundle_cells: set[GridCell] = set()

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
        if nesting == "outward" and exit_dx * (p_i - c_i) < 0 and row_index != 1:
            return None, f"{terminal.id}'s pad column is inward of its corridor lane"
        channel_rank = row_index if nesting == "outward" else row_count + 1 - row_index
        h_i = _river_channel_lane_y(target_cells, config, channel_rank, track_pitch_cells)
        if not (0 <= h_i < obstacle_map.grid.height):
            return None, f"{terminal.id}'s channel lane is off the grid"
        if config.pad_side == "top" and h_i < anchor_cell[1]:
            return None, f"{terminal.id}'s channel lane is below its own terminal row"
        if config.pad_side == "bottom" and h_i > anchor_cell[1]:
            return None, f"{terminal.id}'s channel lane is above its own terminal row"

        # Row 1's inward case (see the docstring) needs no separate
        # construction: the same four-point path, with p_i now on the
        # inward side of c_i, already draws out-to-c_i, up-to-h_i,
        # back-to-p_i, into-the-pad; count_direction_changes below classes
        # it "Z" (2-3 bends) on its own.
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
        bare_cells = set(cells_from_point_path(centerline))
        if bare_cells & own_bundle_cells:
            return (
                None,
                f"{terminal.id}'s river wire crosses another row of its own bundle",
            )
        own_bundle_cells.update(bare_cells)

        bends = count_direction_changes(centerline)
        shape: Literal["L", "Z"] = "L" if bends <= 1 else "Z"
        wires.append((member, centerline, shape))

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
    channel_rank: int,
    track_pitch_cells: int,
) -> int:
    """Channel lane height h = pad access row -+ channel_rank * track pitch.

    Same shelf arithmetic as ``_individual_pad_lane_point``'s
    ``target_edge_y +- shelf_index * track_pitch_cells``, applied directly
    by rank (1 = nearest the pad row) since the river construction already
    knows row order unambiguously, instead of going through that function's
    lane-rank/lane-count/direction_x indirection (built for a different
    caller that did not know row order in advance).

    2026-09-28 (mirrored nesting plan): the caller passes ``row_index``
    itself for "outward" nesting's h_i, or the mirrored rank
    ``row_count + 1 - row_index`` for "inward" nesting's h'_i -- this
    function only ever computes "rank-th shelf from the pad edge", unaware
    of which row that rank belongs to.

    Not clamped to the grid: the caller (``try_river_route_bundle``) refuses
    a lane that ends up off the grid instead of silently sharing a lane with
    another row at the edge.
    """

    if config.pad_side == "top":
        target_edge_y = min(y for _, y in target_cells)
        return target_edge_y - channel_rank * track_pitch_cells
    target_edge_y = max(y for _, y in target_cells)
    return target_edge_y + channel_rank * track_pitch_cells


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
