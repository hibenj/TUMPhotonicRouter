"""Greedy rooted common-bus router for heater terminal selection."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Any, Iterable, cast

from photonic_router.static_obstacle_builder import physical_to_grid

from .port_access import choose_route_start_cell, ordered_route_start_cells
from .types import (
    BBox,
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    GridCell,
    TerminalBusRoute,
    TerminalPairGroup,
)


@dataclass(frozen=True)
class _CandidatePath:
    group: TerminalPairGroup
    terminal: ElectricalTerminal
    path: tuple[GridCell, ...]

    @property
    def cost(self) -> int:
        return max(0, len(self.path) - 1)


def route_common_bus(
    terminal_groups: tuple[TerminalPairGroup, ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> CommonBusRoutingResult:
    """Connect exactly one terminal from each heater to the fixed common bus.

    Three phases run in order, each popping the heaters it serves out of
    ``remaining`` so the next phase only sees what is left:

    1. Column trunks (``_route_column_trunks``, unconditional): heaters whose
       terminals stack in the same grid column (typically several rows of the
       same instance type sharing one x) are served by one straight vertical
       run per column, landing every member's own terminal as a contact along
       the way instead of routing each heater's terminal to the tree
       independently.
    2. Local trunks (``_route_local_trunks``, only under
       ``common_bus_routing_strategy == "local_trunk_then_greedy"``): same-row
       heater pairs get a short two-armed trunk to a shared midpoint column.
    3. The greedy group-Steiner heuristic below: the existing bus stripe is
       the root tree, and at each step the router evaluates both candidate
       terminals of every still-unconnected heater and commits the globally
       cheapest path to the current same-net tree. Because every terminal the
       common bus selects belongs to the same net, a candidate path may pass
       through the open cells of terminals already selected for the bus (by
       either earlier phase or an earlier iteration of this one); only
       terminals not yet selected for the bus remain forbidden ground.
    """

    config.validate()
    remaining = {group.heater_id: group for group in terminal_groups}
    selected: dict[str, ElectricalTerminal] = {}
    unselected: dict[str, ElectricalTerminal] = {}
    routes: list[TerminalBusRoute] = []
    tree_cells: set[GridCell] = set(obstacle_map.bus.cells)
    blocked = set(obstacle_map.blocked_cells)
    all_terminal_cells = _all_terminal_cells(obstacle_map)
    median_x = _terminal_median_grid_x(terminal_groups, obstacle_map)
    local_target_x_by_group = _local_pair_target_grid_x_by_group(
        terminal_groups,
        obstacle_map,
        config,
        fallback_x=median_x,
    )

    _route_column_trunks(
        remaining,
        selected,
        unselected,
        routes,
        tree_cells,
        blocked,
        obstacle_map,
        config,
        local_target_x_by_group,
    )

    if config.common_bus_routing_strategy == "local_trunk_then_greedy":
        _route_local_trunks(
            remaining,
            selected,
            unselected,
            routes,
            tree_cells,
            blocked,
            obstacle_map,
            config,
            local_target_x_by_group,
        )

    while remaining:
        candidates: list[_CandidatePath] = []
        for group in remaining.values():
            for terminal in group.terminals:
                forbidden = _forbidden_terminal_cells(
                    obstacle_map,
                    all_terminal_cells,
                    allowed_terminal_ids={terminal.id} | {t.id for t in selected.values()},
                )
                path = _shortest_path_to_tree(
                    terminal,
                    tree_cells=frozenset(tree_cells),
                    blocked=blocked,
                    forbidden=forbidden,
                    obstacle_map=obstacle_map,
                )
                if path is None:
                    continue
                candidates.append(_CandidatePath(group=group, terminal=terminal, path=path))

        if not candidates:
            break

        best = min(
            candidates,
            key=lambda candidate: (
                _candidate_selection_score(
                    candidate,
                    median_x,
                    local_target_x_by_group,
                    obstacle_map,
                    config,
                ),
                _candidate_target_distance(
                    candidate, median_x, local_target_x_by_group, obstacle_map, config
                ),
                candidate.cost,
                candidate.group.heater_id,
                candidate.terminal.side_key,
                candidate.terminal.id,
                candidate.path,
            ),
        )
        group = best.group
        selected[group.heater_id] = best.terminal
        other_terminal = (
            group.terminal_b if group.terminal_a.id == best.terminal.id else group.terminal_a
        )
        unselected[group.heater_id] = other_terminal
        best_path = cast(tuple[GridCell, ...], best.path)
        routes.append(
            TerminalBusRoute(
                heater_id=group.heater_id,
                terminal=best.terminal,
                path=best_path,
                cost=best.cost,
                access_anchor_cell=_terminal_access_anchor_cell(
                    obstacle_map,
                    best.terminal.id,
                ),
                route_start_cell=best_path[0] if best_path else None,
                used_access_anchor=_route_uses_access_anchor(
                    obstacle_map,
                    best.terminal.id,
                    best_path,
                ),
            )
        )
        tree_cells.update(best_path)
        remaining.pop(group.heater_id)

    return CommonBusRoutingResult(
        bus_side=config.bus_side,
        bus=obstacle_map.bus,
        selected_terminals=selected,
        unselected_terminals=unselected,
        routes=tuple(routes),
        tree_cells=frozenset(tree_cells),
        failed_heaters=tuple(sorted(remaining)),
    )


def _route_column_trunks(
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
    column (``_nearest_terminal_cell``, the rule the local-trunk arms use, so
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

    all_terminal_cells = _all_terminal_cells(obstacle_map)
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
        if side_keys == {"l"}:
            exit_dx = -1
        elif side_keys == {"r"}:
            exit_dx = 1
        else:
            continue

        allowed_cells: set[GridCell] = set()
        for _, terminal, anchor in members:
            allowed_cells.update(_terminal_open_cells(obstacle_map, terminal.id))
            allowed_cells.add(anchor)
        forbidden_cells = all_terminal_cells.difference(allowed_cells)

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
                all_terminal_cells=all_terminal_cells,
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
                vertical = _axis_path(junction, next_junction)
            else:
                vertical = _axis_path(junction, stripe_entry)
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
                    access_anchor_cell=_terminal_access_anchor_cell(
                        obstacle_map,
                        terminal.id,
                    ),
                    route_start_cell=path[0] if path else None,
                    used_access_anchor=_route_uses_access_anchor(
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
    (``_nearest_terminal_cell``) from which the stub runs outward and hits
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
        own_open = set(_terminal_open_cells(obstacle_map, terminal.id))
        own_anchor = _terminal_access_anchor_cell(obstacle_map, terminal.id)
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
            start = _nearest_terminal_cell(terminal, obstacle_map, column)
            if start is None or exit_dx * (column - start[0]) < 0:
                legal = False
                break
            stub = _axis_path(start, (column, start[1]))
            if _path_hits_blockers(
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

        _, top_center_y = _grid_cell_center_um(top_junction, grid)
        stripe_edge_um = bus.bbox[3] if bus_side == "bottom" else bus.bbox[1]
        band_y0 = min(top_center_y, stripe_edge_um)
        band_y1 = max(top_center_y, stripe_edge_um)
        center_x, _ = _grid_cell_center_um((column, 0), grid)
        band_bbox = (center_x - half_band, band_y0, center_x + half_band, band_y1)
        if _wire_band_hits_raw_obstacles(band_bbox, raw_obstacle_bboxes):
            continue

        vertical_run = _axis_path(top_junction, stripe_entry)
        if _path_hits_blockers(
            vertical_run,
            blocked,
            allowed=vertical_allowed,
            forbidden=forbidden_cells,
        ):
            continue

        return column, stripe_entry, stubs, junctions

    return None


def _grid_cell_center_um(cell: GridCell, grid: Any) -> tuple[float, float]:
    origin_x, origin_y = grid.origin
    grid_size = grid.grid_size_um
    return (
        origin_x + (cell[0] + 0.5) * grid_size,
        origin_y + (cell[1] + 0.5) * grid_size,
    )


def _wire_band_hits_raw_obstacles(
    band_bbox: BBox,
    raw_obstacle_bboxes: tuple[BBox, ...],
) -> bool:
    return any(
        _rect_intersects_positive_area(band_bbox, obstacle) for obstacle in raw_obstacle_bboxes
    )


def _rect_intersects_positive_area(left: BBox, right: BBox) -> bool:
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


def _route_local_trunks(
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
    pairs = _local_same_row_pairs(tuple(remaining.values()), config)
    used: set[str] = set()
    for left_id, right_id in pairs:
        if left_id in used or right_id in used:
            continue
        left_group = remaining.get(left_id)
        right_group = remaining.get(right_id)
        if left_group is None or right_group is None:
            continue
        left_pair, right_pair = sorted(
            (left_group, right_group),
            key=lambda group: _group_center(group)[0],
        )
        group_pair = (left_pair, right_pair)
        target_grid_x = int(
            round(
                (
                    local_target_x_by_group.get(group_pair[0].heater_id, 0.0)
                    + local_target_x_by_group.get(group_pair[1].heater_id, 0.0)
                )
                / 2.0
            )
        )
        pair_routes = _build_local_trunk_pair_routes(
            group_pair,
            target_grid_x,
            tree_cells=frozenset(tree_cells),
            blocked=blocked,
            obstacle_map=obstacle_map,
            config=config,
        )
        if pair_routes is None:
            continue
        for group, terminal, path in pair_routes:
            selected[group.heater_id] = terminal
            other_terminal = (
                group.terminal_b if group.terminal_a.id == terminal.id else group.terminal_a
            )
            unselected[group.heater_id] = other_terminal
            routes.append(
                TerminalBusRoute(
                    heater_id=group.heater_id,
                    terminal=terminal,
                    path=path,
                    cost=max(0, len(path) - 1),
                    access_anchor_cell=_terminal_access_anchor_cell(
                        obstacle_map,
                        terminal.id,
                    ),
                    route_start_cell=path[0] if path else None,
                    used_access_anchor=_route_uses_access_anchor(
                        obstacle_map,
                        terminal.id,
                        path,
                    ),
                )
            )
            tree_cells.update(path)
            remaining.pop(group.heater_id, None)
            used.add(group.heater_id)


def _local_same_row_pairs(
    terminal_groups: tuple[TerminalPairGroup, ...],
    config: ElectricalRoutingConfig,
) -> tuple[tuple[str, str], ...]:
    centers = {group.heater_id: _group_center(group) for group in terminal_groups}
    pairs: list[tuple[float, str, str]] = []
    for group in terminal_groups:
        x, y = centers[group.heater_id]
        candidates: list[tuple[float, str]] = []
        for other in terminal_groups:
            if other.heater_id == group.heater_id:
                continue
            other_x, other_y = centers[other.heater_id]
            gap = abs(other_x - x)
            if gap <= 0 or gap > config.common_bus_local_pair_max_gap_um:
                continue
            if abs(other_y - y) > config.common_bus_local_pair_y_tolerance_um:
                continue
            candidates.append((gap, other.heater_id))
        if not candidates:
            continue
        gap, other_id = min(candidates)
        left_id, right_id = sorted(
            (group.heater_id, other_id),
            key=lambda heater_id: centers[heater_id][0],
        )
        pairs.append((gap, left_id, right_id))
    unique: dict[tuple[str, str], float] = {}
    for gap, left_id, right_id in pairs:
        key = (left_id, right_id)
        unique[key] = min(gap, unique.get(key, gap))
    return tuple(key for key, _ in sorted(unique.items(), key=lambda item: (item[1], item[0])))


def _build_local_trunk_pair_routes(
    group_pair: tuple[TerminalPairGroup, TerminalPairGroup],
    target_grid_x: int,
    *,
    tree_cells: frozenset[GridCell],
    blocked: set[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[tuple[TerminalPairGroup, ElectricalTerminal, tuple[GridCell, ...]], ...] | None:
    terminal_routes: list[tuple[TerminalPairGroup, ElectricalTerminal, tuple[GridCell, ...]]] = []
    arm_endpoints: list[GridCell] = []
    all_terminal_cells = _all_terminal_cells(obstacle_map)
    for group in group_pair:
        terminal = min(
            group.terminals,
            key=lambda candidate: (
                abs(
                    physical_to_grid(candidate.center[0], candidate.center[1], obstacle_map.grid)[0]
                    - target_grid_x
                ),
                candidate.side_key,
                candidate.id,
            ),
        )
        start = _nearest_terminal_cell(terminal, obstacle_map, target_grid_x)
        if start is None:
            return None
        _, start_y = start
        trunk_cell = (target_grid_x, start_y)
        arm = _axis_path(start, trunk_cell)
        allowed_terminal_cells = set(_terminal_open_cells(obstacle_map, terminal.id))
        access_anchor = _terminal_access_anchor_cell(obstacle_map, terminal.id)
        if access_anchor is not None:
            allowed_terminal_cells.add(access_anchor)
        forbidden = set(all_terminal_cells).difference(allowed_terminal_cells)
        if _path_hits_blockers(
            arm,
            blocked,
            allowed=allowed_terminal_cells,
            forbidden=forbidden,
        ):
            return None
        terminal_routes.append((group, terminal, arm))
        arm_endpoints.append(trunk_cell)

    trunk_y_values = [cell[1] for cell in arm_endpoints]
    tree_targets = tree_cells.difference(all_terminal_cells) or tree_cells
    target_tree_cell = min(tree_targets, key=lambda cell: (abs(cell[0] - target_grid_x), cell[1]))
    trunk_bus_cell = (target_grid_x, target_tree_cell[1])
    trunk_y_values.append(trunk_bus_cell[1])
    trunk_min_y = min(trunk_y_values)
    trunk_max_y = max(trunk_y_values)
    trunk = tuple((target_grid_x, y) for y in range(trunk_min_y, trunk_max_y + 1))
    if _path_hits_blockers(
        trunk,
        blocked,
        allowed=set(arm_endpoints),
        forbidden=set(all_terminal_cells).difference(arm_endpoints),
    ):
        return None
    connector = _axis_path(trunk_bus_cell, target_tree_cell)
    if _path_hits_blockers(
        connector,
        blocked,
        allowed=set(tree_cells),
        forbidden=set(all_terminal_cells),
    ):
        return None

    trunk_and_connector = tuple(dict.fromkeys((*trunk, *connector)))
    result: list[tuple[TerminalPairGroup, ElectricalTerminal, tuple[GridCell, ...]]] = []
    for index, (group, terminal, arm) in enumerate(terminal_routes):
        if index == 0:
            path = tuple(dict.fromkeys((*arm, *trunk_and_connector)))
        else:
            path = tuple(dict.fromkeys((*arm, *trunk)))
        result.append((group, terminal, path))
    return tuple(result)


def _nearest_terminal_cell(
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
    target_grid_x: int,
) -> GridCell | None:
    center = physical_to_grid(terminal.center[0], terminal.center[1], obstacle_map.grid)
    choice = choose_route_start_cell(
        access=_terminal_access(obstacle_map, terminal.id),
        opened_cells=_terminal_open_cells(obstacle_map, terminal.id),
        grid=obstacle_map.grid,
        fallback_cell=center,
        bias_key=lambda cell: (abs(cell[0] - target_grid_x), abs(cell[1]), cell[0], cell[1]),
        prefer_access_anchor=False,
    )
    return choice.cell if choice is not None else None


def _axis_path(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
    x0, y0 = start
    x1, y1 = end
    cells: list[GridCell] = []
    step_x = 1 if x1 >= x0 else -1
    for x in range(x0, x1 + step_x, step_x):
        cells.append((x, y0))
    step_y = 1 if y1 >= y0 else -1
    for y in range(y0 + step_y, y1 + step_y, step_y):
        cells.append((x1, y))
    return tuple(cells)


def _path_hits_blockers(
    path: Iterable[GridCell],
    blocked: set[GridCell],
    *,
    allowed: set[GridCell],
    forbidden: set[GridCell] | frozenset[GridCell] = frozenset(),
) -> bool:
    return any((cell in blocked and cell not in allowed) or cell in forbidden for cell in path)


def _local_pair_target_grid_x_by_group(
    terminal_groups: tuple[TerminalPairGroup, ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    fallback_x: float,
) -> dict[str, float]:
    if config.common_bus_terminal_selection != "local_pair_median_x_biased":
        return {group.heater_id: fallback_x for group in terminal_groups}

    group_centers = {group.heater_id: _group_center(group) for group in terminal_groups}
    target_by_group: dict[str, float] = {}
    max_gap = config.common_bus_local_pair_max_gap_um
    y_tol = config.common_bus_local_pair_y_tolerance_um
    for group in terminal_groups:
        x, y = group_centers[group.heater_id]
        neighbors: list[tuple[float, str, float]] = []
        for other in terminal_groups:
            if other.heater_id == group.heater_id:
                continue
            other_x, other_y = group_centers[other.heater_id]
            x_gap = abs(other_x - x)
            if x_gap <= 0 or x_gap > max_gap:
                continue
            if abs(other_y - y) > y_tol:
                continue
            neighbors.append((x_gap, other.heater_id, other_x))
        if not neighbors:
            target_by_group[group.heater_id] = fallback_x
            continue
        _, _, neighbor_x = min(neighbors)
        target_x_um = (x + neighbor_x) / 2.0
        target_grid_x, _ = physical_to_grid(target_x_um, y, obstacle_map.grid)
        target_by_group[group.heater_id] = float(target_grid_x)
    return target_by_group


def _group_center(group: TerminalPairGroup) -> tuple[float, float]:
    return (
        (group.terminal_a.center[0] + group.terminal_b.center[0]) / 2.0,
        (group.terminal_a.center[1] + group.terminal_b.center[1]) / 2.0,
    )


def _terminal_median_grid_x(
    terminal_groups: tuple[TerminalPairGroup, ...],
    obstacle_map: ElectricalObstacleMap,
) -> float:
    terminal_grid_xs = [
        physical_to_grid(terminal.center[0], terminal.center[1], obstacle_map.grid)[0]
        for group in terminal_groups
        for terminal in group.terminals
    ]
    if not terminal_grid_xs:
        return 0.0
    return float(median(terminal_grid_xs))


def _candidate_selection_score(
    candidate: _CandidatePath,
    median_x: float,
    local_target_x_by_group: dict[str, float],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> float:
    if config.common_bus_terminal_selection == "path_cost":
        return float(candidate.cost)
    return float(candidate.cost) + (
        config.common_bus_median_bias_weight
        * _candidate_target_distance(
            candidate,
            median_x,
            local_target_x_by_group,
            obstacle_map,
            config,
        )
    )


def _candidate_target_distance(
    candidate: _CandidatePath,
    median_x: float,
    local_target_x_by_group: dict[str, float],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> float:
    if config.common_bus_terminal_selection == "path_cost":
        return 0.0
    if config.common_bus_terminal_selection == "local_pair_median_x_biased":
        target_x = local_target_x_by_group.get(candidate.group.heater_id, median_x)
    else:
        target_x = median_x
    return _candidate_distance_to_target_x(candidate, target_x, obstacle_map)


def _candidate_median_distance(
    candidate: _CandidatePath,
    median_x: float,
    obstacle_map: ElectricalObstacleMap,
) -> float:
    return _candidate_distance_to_target_x(candidate, median_x, obstacle_map)


def _candidate_distance_to_target_x(
    candidate: _CandidatePath,
    target_x: float,
    obstacle_map: ElectricalObstacleMap,
) -> float:
    grid_x, _ = physical_to_grid(
        candidate.terminal.center[0],
        candidate.terminal.center[1],
        obstacle_map.grid,
    )
    return abs(float(grid_x) - target_x)


def _all_terminal_cells(obstacle_map: ElectricalObstacleMap) -> frozenset[GridCell]:
    cells: set[GridCell] = set()
    terminal_open_cells = (
        obstacle_map.common_bus_terminal_open_cells or obstacle_map.terminal_open_cells
    )
    for terminal_cells in terminal_open_cells.values():
        cells.update(terminal_cells)
    return frozenset(cells)


def _forbidden_terminal_cells(
    obstacle_map: ElectricalObstacleMap,
    all_terminal_cells: frozenset[GridCell],
    *,
    allowed_terminal_ids: set[str],
) -> frozenset[GridCell]:
    allowed: set[GridCell] = set()
    for terminal_id in allowed_terminal_ids:
        allowed.update(_terminal_open_cells(obstacle_map, terminal_id))
    return frozenset(all_terminal_cells.difference(allowed))


def _shortest_path_to_tree(
    terminal: ElectricalTerminal,
    *,
    tree_cells: frozenset[GridCell],
    blocked: set[GridCell],
    forbidden: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
) -> tuple[GridCell, ...] | None:
    grid = obstacle_map.grid
    terminal_cells = _terminal_open_cells(obstacle_map, terminal.id)
    center_cell = physical_to_grid(terminal.center[0], terminal.center[1], grid)
    starts = ordered_route_start_cells(
        access=_terminal_access(obstacle_map, terminal.id),
        opened_cells=terminal_cells,
        grid=grid,
        fallback_cell=center_cell,
        bias_key=lambda cell: (_manhattan(cell, center_cell), cell[0], cell[1]),
    )
    if not starts:
        return None

    targets = tree_cells.difference(forbidden)
    if not targets:
        return None
    if any(start in targets for start in starts):
        start = min((cell for cell in starts if cell in targets), key=lambda c: (c[0], c[1]))
        return (start,)

    queue: deque[GridCell] = deque()
    parent: dict[GridCell, GridCell | None] = {}
    for start in starts:
        if start in forbidden:
            continue
        parent[start] = None
        queue.append(start)

    while queue:
        current = queue.popleft()
        for neighbor in _neighbors(current, bus_side=obstacle_map.bus.side):
            if not _in_bounds(neighbor, grid.width, grid.height):
                continue
            if neighbor in parent:
                continue
            if neighbor in forbidden:
                continue
            if neighbor not in targets and neighbor in blocked:
                continue

            parent[neighbor] = current
            if neighbor in targets:
                return _reconstruct_path(parent, neighbor)
            queue.append(neighbor)

    return None


def _neighbors(cell: GridCell, *, bus_side: str) -> tuple[GridCell, GridCell, GridCell, GridCell]:
    x, y = cell
    if bus_side == "bottom":
        return ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1))
    return ((x, y + 1), (x - 1, y), (x + 1, y), (x, y - 1))


def _reconstruct_path(
    parent: dict[GridCell, GridCell | None],
    end: GridCell,
) -> tuple[GridCell, ...]:
    path: list[GridCell] = []
    current: GridCell | None = end
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return tuple(path)


def _in_bounds(cell: GridCell, width: int, height: int) -> bool:
    x, y = cell
    return 0 <= x < width and 0 <= y < height


def _manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> frozenset[GridCell]:
    if obstacle_map.common_bus_terminal_open_cells:
        return obstacle_map.common_bus_terminal_open_cells.get(terminal_id, frozenset())
    return obstacle_map.terminal_open_cells.get(terminal_id, frozenset())


def _terminal_access_anchor_cell(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> GridCell | None:
    access = _terminal_access(obstacle_map, terminal_id)
    return access.anchor_cell if access is not None else None


def _terminal_access(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
):
    return obstacle_map.common_bus_port_accesses.get(terminal_id)


def _route_uses_access_anchor(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
    path: tuple[GridCell, ...],
) -> bool:
    anchor = _terminal_access_anchor_cell(obstacle_map, terminal_id)
    return bool(path) and anchor is not None and path[0] == anchor
