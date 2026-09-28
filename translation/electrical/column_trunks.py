"""Straight column-trunk phase of the common bus router."""

from __future__ import annotations

from photonic_router.static_obstacle_builder import physical_to_grid

from .bus_search import (
    all_terminal_cells,
    axis_path,
    nearest_terminal_cell,
    path_hits_blockers,
    route_uses_access_anchor,
    terminal_access_anchor_cell,
    terminal_open_cells,
)
from .pitch_grid import grid_cell_center_um
from .types import (
    BBox,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    GridCell,
    TerminalBusRoute,
    TerminalPairGroup,
    terminal_exit_dx,
)


def route_column_trunks(
    remaining: dict[str, TerminalPairGroup],
    selected: dict[str, ElectricalTerminal],
    unselected: dict[str, ElectricalTerminal],
    routes: list[TerminalBusRoute],
    tree_cells: set[GridCell],
    blocked: set[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    local_target_x_by_group: dict[str, float],
) -> None:
    """Serve column groups of bus terminals with straight vertical trunks.

    A terminal's column is the x of its access anchor cell
    (``obstacle_map.common_bus_port_accesses[terminal.id].anchor_cell``).
    Columns are clustered (a new cluster whenever consecutive distinct
    columns are more than one cell apart); a cluster holding terminals of at
    least two heaters is a group, with its most frequent anchor column as the
    reference column. A heater with a terminal in exactly one group selects
    that terminal; with terminals in two groups it selects the larger group's
    (ties: nearer ``local_target_x_by_group``, then side_key, then id); a
    heater with neither is left to the later phases.

    The trunk runs beside the heater bodies, not through them: for a group
    the exit direction comes from the members' shared side_key (``l`` -1,
    ``r`` +1; a group with mixed sides is not trunked) and the trunk column
    is the first legal one of ``reference + exit_dx * k`` for k = 0..8
    (``_find_legal_column_trunk``). Each member joins the trunk with a
    perpendicular stub that starts at its own open cell nearest the trunk
    column (``nearest_terminal_cell``, the rule the local-trunk arms use, so
    the port adapter leaves in the port's real exit direction); the trunk
    runs from the topmost member's junction straight to the bus stripe. If
    no column is legal from the topmost member, that member is dropped and
    the search retried from the next one; members above the first legal
    start stay in ``remaining`` for the later phases.

    Each served member gets one ``TerminalBusRoute``: its stub followed by
    the vertical run to the next lower member's junction (or the stripe entry
    for the lowest member), so every route has at most one direction change
    and the merged metal is one trunk with one contact per terminal.
    """

    terminal_cluster, cluster_trunk_column, cluster_heater_ids = _column_groups(
        remaining, obstacle_map
    )
    if not terminal_cluster:
        return

    all_terminal_cells_set = all_terminal_cells(obstacle_map)
    bus_side = obstacle_map.bus.side

    picks: dict[str, tuple[ElectricalTerminal, int]] = {}
    for heater_id, group in remaining.items():
        pick = _select_column_terminal(
            group,
            terminal_cluster,
            cluster_heater_ids,
            local_target_x_by_group,
            obstacle_map,
        )
        if pick is not None:
            picks[heater_id] = pick

    routing_groups: dict[int, list[str]] = {}
    for heater_id, (_, cluster_idx) in picks.items():
        routing_groups.setdefault(cluster_idx, []).append(heater_id)

    for cluster_idx, heater_ids in routing_groups.items():
        anchor_column = cluster_trunk_column[cluster_idx]
        members: list[tuple[str, ElectricalTerminal, GridCell]] = []
        for heater_id in heater_ids:
            terminal, _ = picks[heater_id]
            access = obstacle_map.common_bus_port_accesses.get(terminal.id)
            if access is None:
                continue
            members.append((heater_id, terminal, access.anchor_cell))
        if not members:
            continue

        members.sort(key=lambda item: item[2][1], reverse=(bus_side == "bottom"))

        side_keys = {terminal.side_key for _, terminal, _ in members}
        if len(side_keys) != 1:
            continue
        exit_dx = terminal_exit_dx(next(iter(side_keys)))
        if exit_dx == 0:
            continue

        allowed_cells: set[GridCell] = set()
        for _, terminal, anchor in members:
            allowed_cells.update(terminal_open_cells(obstacle_map, terminal.id))
            allowed_cells.add(anchor)
        forbidden_cells = all_terminal_cells_set.difference(allowed_cells)

        start_index = 0
        resolution: (
            tuple[int, GridCell, dict[str, tuple[GridCell, ...]], dict[str, GridCell]] | None
        ) = None
        while start_index < len(members):
            resolution = _find_legal_column_trunk(
                members[start_index:],
                anchor_column,
                exit_dx,
                allowed_cells=allowed_cells,
                forbidden_cells=forbidden_cells,
                blocked=blocked,
                tree_cells=tree_cells,
                all_terminal_cells=all_terminal_cells_set,
                obstacle_map=obstacle_map,
                config=config,
            )
            if resolution is not None:
                break
            start_index += 1

        if resolution is None:
            continue

        _trunk_column, stripe_entry, stubs, junctions = resolution
        served_members = members[start_index:]
        for index, (heater_id, terminal, _) in enumerate(served_members):
            stub = stubs[heater_id]
            junction = junctions[heater_id]
            if index + 1 < len(served_members):
                next_junction = junctions[served_members[index + 1][0]]
                vertical = axis_path(junction, next_junction)
            else:
                vertical = axis_path(junction, stripe_entry)
            path = tuple(dict.fromkeys((*stub, *vertical)))
            group = remaining[heater_id]
            selected[heater_id] = terminal
            other_terminal = (
                group.terminal_b if group.terminal_a.id == terminal.id else group.terminal_a
            )
            unselected[heater_id] = other_terminal
            routes.append(
                TerminalBusRoute(
                    heater_id=heater_id,
                    terminal=terminal,
                    path=path,
                    cost=max(0, len(path) - 1),
                    access_anchor_cell=terminal_access_anchor_cell(
                        obstacle_map,
                        terminal.id,
                    ),
                    route_start_cell=path[0] if path else None,
                    used_access_anchor=route_uses_access_anchor(
                        obstacle_map,
                        terminal.id,
                        path,
                    ),
                )
            )
            tree_cells.update(path)
            remaining.pop(heater_id, None)


def _find_legal_column_trunk(
    members: list[tuple[str, ElectricalTerminal, GridCell]],
    anchor_column: int,
    exit_dx: int,
    *,
    allowed_cells: set[GridCell],
    forbidden_cells: frozenset[GridCell],
    blocked: set[GridCell],
    tree_cells: set[GridCell],
    all_terminal_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[int, GridCell, dict[str, tuple[GridCell, ...]], dict[str, GridCell]] | None:
    """Return ``(trunk_column, stripe_entry, stubs, junctions)`` for the first legal k in 0..8.

    A candidate column is legal when every member has a stub start
    (``nearest_terminal_cell``) from which the stub runs outward and hits
    neither ``blocked`` nor another terminal's open cells; when the column
    has a bus stripe cell; when the trunk's wire band (half the wire width
    plus the obstacle clearance, from the topmost junction to the stripe
    edge) overlaps no raw obstacle box; and when the trunk's cells hit
    neither ``blocked`` nor ``forbidden_cells``.
    """

    grid = obstacle_map.grid
    bus = obstacle_map.bus
    bus_side = bus.side
    half_band = config.wire_width_um / 2.0 + config.obstacle_clearance_um
    vertical_allowed = allowed_cells | tree_cells
    raw_obstacle_bboxes = obstacle_map.raw_obstacle_bboxes

    member_stub_allowed: dict[str, frozenset[GridCell]] = {}
    member_stub_forbidden: dict[str, frozenset[GridCell]] = {}
    for heater_id, terminal, _ in members:
        own_open = set(terminal_open_cells(obstacle_map, terminal.id))
        own_anchor = terminal_access_anchor_cell(obstacle_map, terminal.id)
        if own_anchor is not None:
            own_open.add(own_anchor)
        member_stub_allowed[heater_id] = frozenset(own_open)
        member_stub_forbidden[heater_id] = all_terminal_cells.difference(own_open)

    for k in range(9):
        column = anchor_column + exit_dx * k

        legal = True
        stubs: dict[str, tuple[GridCell, ...]] = {}
        junctions: dict[str, GridCell] = {}
        for heater_id, terminal, _ in members:
            start = nearest_terminal_cell(terminal, obstacle_map, column)
            if start is None or exit_dx * (column - start[0]) < 0:
                legal = False
                break
            stub = axis_path(start, (column, start[1]))
            if path_hits_blockers(
                stub,
                blocked,
                allowed=member_stub_allowed[heater_id],
                forbidden=member_stub_forbidden[heater_id],
            ):
                legal = False
                break
            stubs[heater_id] = stub
            junctions[heater_id] = (column, start[1])
        if not legal:
            continue

        top_junction = junctions[members[0][0]]
        nearest_member_y = junctions[members[-1][0]][1]

        stripe_candidates = tuple(cell for cell in bus.cells if cell[0] == column)
        if not stripe_candidates:
            continue
        stripe_entry = min(
            stripe_candidates,
            key=lambda cell: (abs(cell[1] - nearest_member_y), cell[1]),
        )

        _, top_center_y = grid_cell_center_um(top_junction, grid)
        stripe_edge_um = bus.bbox[3] if bus_side == "bottom" else bus.bbox[1]
        band_y0 = min(top_center_y, stripe_edge_um)
        band_y1 = max(top_center_y, stripe_edge_um)
        center_x, _ = grid_cell_center_um((column, 0), grid)
        band_bbox = (center_x - half_band, band_y0, center_x + half_band, band_y1)
        if _wire_band_hits_raw_obstacles(band_bbox, raw_obstacle_bboxes):
            continue

        vertical_run = axis_path(top_junction, stripe_entry)
        if path_hits_blockers(
            vertical_run,
            blocked,
            allowed=vertical_allowed,
            forbidden=forbidden_cells,
        ):
            continue

        return column, stripe_entry, stubs, junctions

    return None


def straight_drop_to_bus(
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    tree_cells: frozenset[GridCell],
    blocked: set[GridCell],
) -> tuple[GridCell, ...] | None:
    """Return a straight branch from ``terminal`` to the bus stripe, or None.

    The single-terminal case of the column trunk: a stub from the terminal's
    open cell nearest the drop column, then one vertical run to the stripe,
    in the first legal column outward from the terminal's exit side (the same
    legality rules as ``_find_legal_column_trunk``). Used for a lone heater
    whose bus terminal was chosen after the bus tree was built (the pad-side
    swap), where the greedy search would otherwise join the nearest branch
    sideways.
    """

    access = obstacle_map.common_bus_port_accesses.get(terminal.id)
    if access is None:
        return None
    exit_dx = terminal_exit_dx(terminal.side_key)
    if exit_dx == 0:
        return None
    all_terminal_cells_set = all_terminal_cells(obstacle_map)
    allowed_cells = set(terminal_open_cells(obstacle_map, terminal.id))
    allowed_cells.add(access.anchor_cell)
    resolution = _find_legal_column_trunk(
        [(terminal.heater_id, terminal, access.anchor_cell)],
        access.anchor_cell[0],
        exit_dx,
        allowed_cells=allowed_cells,
        forbidden_cells=all_terminal_cells_set.difference(allowed_cells),
        blocked=blocked,
        tree_cells=set(tree_cells),
        all_terminal_cells=all_terminal_cells_set,
        obstacle_map=obstacle_map,
        config=config,
    )
    if resolution is None:
        return None
    _column, stripe_entry, stubs, junctions = resolution
    stub = stubs[terminal.heater_id]
    vertical = axis_path(junctions[terminal.heater_id], stripe_entry)
    return tuple(dict.fromkeys((*stub, *vertical)))


def _wire_band_hits_raw_obstacles(
    band_bbox: BBox,
    raw_obstacle_bboxes: tuple[BBox, ...],
) -> bool:
    return any(
        _rect_intersects_positive_area(band_bbox, obstacle) for obstacle in raw_obstacle_bboxes
    )


def _rect_intersects_positive_area(left: BBox, right: BBox) -> bool:
    # Not rect_geometry.rect_intersects: that predicate treats two rects
    # touching at an edge (a shared boundary, zero-area overlap) as
    # intersecting, which would refuse a trunk column merely adjacent to an
    # obstacle's edge; this wire-band check needs strictly positive overlap
    # area.
    x0 = max(left[0], right[0])
    y0 = max(left[1], right[1])
    x1 = min(left[2], right[2])
    y1 = min(left[3], right[3])
    return x1 > x0 and y1 > y0


def _column_groups(
    remaining: dict[str, TerminalPairGroup],
    obstacle_map: ElectricalObstacleMap,
) -> tuple[dict[str, int], dict[int, int], dict[int, set[str]]]:
    """Cluster terminal anchor columns; return only clusters that qualify as groups.

    Returns ``(terminal_cluster, cluster_trunk_column, cluster_heater_ids)``:
    ``terminal_cluster`` maps a terminal id to its cluster index, for
    terminals whose column cluster holds at least two distinct heaters;
    ``cluster_trunk_column`` maps that cluster index to its trunk column (the
    most frequent anchor column in the cluster, ties toward the smaller);
    ``cluster_heater_ids`` maps it to the set of distinct heater ids with a
    terminal in that cluster.
    """

    columns: list[tuple[int, str, str]] = []
    for group in remaining.values():
        for terminal in group.terminals:
            access = obstacle_map.common_bus_port_accesses.get(terminal.id)
            if access is None:
                continue
            columns.append((access.anchor_cell[0], group.heater_id, terminal.id))
    if not columns:
        return {}, {}, {}

    distinct_cols = sorted({col for col, _, _ in columns})
    cluster_of_col: dict[int, int] = {}
    cluster_index = -1
    prev_col: int | None = None
    for col in distinct_cols:
        if prev_col is None or col - prev_col > 1:
            cluster_index += 1
        cluster_of_col[col] = cluster_index
        prev_col = col

    cluster_heater_ids: dict[int, set[str]] = {}
    cluster_col_counts: dict[int, dict[int, int]] = {}
    terminal_cluster_all: dict[str, int] = {}
    for col, heater_id, terminal_id in columns:
        cluster_idx = cluster_of_col[col]
        terminal_cluster_all[terminal_id] = cluster_idx
        cluster_heater_ids.setdefault(cluster_idx, set()).add(heater_id)
        counts = cluster_col_counts.setdefault(cluster_idx, {})
        counts[col] = counts.get(col, 0) + 1

    cluster_trunk_column: dict[int, int] = {}
    for cluster_idx, counts in cluster_col_counts.items():
        cluster_trunk_column[cluster_idx] = min(counts, key=lambda c: (-counts[c], c))

    qualifying = {idx for idx, heater_ids in cluster_heater_ids.items() if len(heater_ids) >= 2}
    terminal_cluster = {
        terminal_id: idx for terminal_id, idx in terminal_cluster_all.items() if idx in qualifying
    }
    cluster_trunk_column = {
        idx: col for idx, col in cluster_trunk_column.items() if idx in qualifying
    }
    cluster_heater_ids = {idx: ids for idx, ids in cluster_heater_ids.items() if idx in qualifying}
    return terminal_cluster, cluster_trunk_column, cluster_heater_ids


def _select_column_terminal(
    group: TerminalPairGroup,
    terminal_cluster: dict[str, int],
    cluster_heater_ids: dict[int, set[str]],
    local_target_x_by_group: dict[str, float],
    obstacle_map: ElectricalObstacleMap,
) -> tuple[ElectricalTerminal, int] | None:
    candidates = [
        (terminal, terminal_cluster[terminal.id])
        for terminal in group.terminals
        if terminal.id in terminal_cluster
    ]
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    target_x = local_target_x_by_group.get(group.heater_id, 0.0)

    def score(item: tuple[ElectricalTerminal, int]) -> tuple[int, float, str, str]:
        terminal, cluster_idx = item
        size = len(cluster_heater_ids.get(cluster_idx, ()))
        grid_x, _ = physical_to_grid(terminal.center[0], terminal.center[1], obstacle_map.grid)
        distance = abs(float(grid_x) - target_x)
        return (-size, distance, terminal.side_key, terminal.id)

    return min(candidates, key=score)
