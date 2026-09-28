"""Stage 7: reconcile a lone heater's bus/pad-wire roles with its pad side."""

from __future__ import annotations

from dataclasses import replace

from .bus_search import (
    all_terminal_cells,
    forbidden_terminal_cells,
    route_uses_access_anchor,
    shortest_path_to_tree,
    terminal_access_anchor_cell,
)
from .column_trunks import straight_drop_to_bus
from .common_bus_router import trim_common_bus_to_connections
from .individual_topology import compute_individual_escape_topology
from .pad_slots import plan_pad_slots
from .types import (
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    GridCell,
    IndividualEscapeTopologyResult,
    PadPlan,
    PadSideReconciliation,
    TerminalBusRoute,
    terminal_exit_dx,
)
from .wire_geometry import dilate_cells, wire_reservation_radius_cells


def reconcile_pad_sides(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    individual_topology: IndividualEscapeTopologyResult | None,
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
) -> PadSideReconciliation:
    """Swap a lone heater's roles when its individual exit points away from its pad.

    Recomputes the bus trim, escape topology and pad plan from the swap so the
    later stages see a consistent set of objects; a run with no mismatch
    returns every input unchanged.
    """

    if common_bus.success and individual_topology is not None and pad_plan is not None:
        swap = _apply_pad_side_consistency_swap(obstacle_map, common_bus, pad_plan, config)
        if swap is not None:
            common_bus, swapped_heater_ids = swap
            obstacle_map, common_bus = trim_common_bus_to_connections(
                obstacle_map,
                common_bus,
                config,
            )
            individual_topology = compute_individual_escape_topology(
                obstacle_map, common_bus, config
            )
            pad_plan = plan_pad_slots(common_bus, obstacle_map, config, individual_topology)
            return PadSideReconciliation(
                obstacle_map=obstacle_map,
                common_bus=common_bus,
                individual_topology=individual_topology,
                pad_plan=pad_plan,
                swapped_heater_ids=swapped_heater_ids,
            )

    return PadSideReconciliation(
        obstacle_map=obstacle_map,
        common_bus=common_bus,
        individual_topology=individual_topology,
        pad_plan=pad_plan,
        swapped_heater_ids=(),
    )


def _apply_pad_side_consistency_swap(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> tuple[CommonBusRoutingResult, tuple[str, ...]] | None:
    """Swap a lone heater's bus/pad-wire roles when its exit points away from its pad.

    2026-09-25 (electrical routing quality plan, Milestone 2): a heater's two
    terminals are interchangeable for the common bus, so ``route_common_bus``
    picks whichever is cheapest to the tree without knowing which one will
    later get the individual pad wire, or which side of that terminal's own
    column its pad will land on. When the individual terminal's physical
    exit direction (``:l`` -> left, ``:r`` -> right) points away from its
    assigned pad's column, that pad wire has to run back through the
    terminal's own corridor to reach it -- and that run walls the corridor
    off for every other bundle that also needs to pass through it (see the
    plan's Milestone 2 surprise on heater_extra_0). Swapping which terminal
    carries the bus branch removes that: the pad wire then exits straight
    toward its own pad. Heaters whose bus terminal shares a grid column with
    another heater's (a Milestone 3 bus column trunk) are left untouched --
    that shared column is the trunk's business, not this heater's alone.
    One deterministic pass: every mismatch is found from the current
    (pre-swap) plan, each reroute attempt reuses the same terminal-to-tree
    search ``route_common_bus`` uses, and a heater keeps its original
    assignment if that reroute fails. Returns the new common-bus result and
    the swapped heater ids, or None if no heater was swapped.
    """

    grouped_heater_ids = _bus_column_group_heater_ids(obstacle_map, common_bus)
    pad_column_um_by_terminal_id = {
        assignment.terminal.id: assignment.slot.center[0]
        for assignment in pad_plan.assignments
        if assignment.kind == "individual" and assignment.terminal is not None
    }
    all_terminal_cells_set = all_terminal_cells(obstacle_map)
    # Dilated by the branch wire's own reservation radius: shortest_path_to_tree
    # only checks the bare centerline against `blocked`, and the branch is
    # realized with untrimmed bend corners (trim_route_tail_bends=False), so a
    # bend placed right at an obstacle's edge can still draw metal that clips
    # it. The router's own route_common_bus loop never hit this because no
    # bend it chose ever landed that close; a swap-driven reroute is not
    # guaranteed the same luck.
    reservation_radius = wire_reservation_radius_cells(obstacle_map, config)
    blocked = dilate_cells(
        set(obstacle_map.blocked_cells),
        reservation_radius,
        obstacle_map.grid.width,
        obstacle_map.grid.height,
    )

    new_selected = dict(common_bus.selected_terminals)
    new_unselected = dict(common_bus.unselected_terminals)
    routes_by_heater_id = {route.heater_id: route for route in common_bus.routes}
    swapped_heater_ids: list[str] = []

    for heater_id, individual_terminal in common_bus.unselected_terminals.items():
        if heater_id in grouped_heater_ids:
            continue
        exit_dx = terminal_exit_dx(individual_terminal.side_key)
        pad_column_um = pad_column_um_by_terminal_id.get(individual_terminal.id)
        if exit_dx == 0 or pad_column_um is None:
            continue
        if exit_dx * (pad_column_um - individual_terminal.center[0]) >= 0:
            continue  # exit already points toward the pad's column.

        bus_terminal = common_bus.selected_terminals[heater_id]
        forbidden = forbidden_terminal_cells(
            obstacle_map,
            all_terminal_cells_set,
            allowed_terminal_ids={individual_terminal.id},
        )
        tree_without_own = frozenset(
            _bus_tree_cells_excluding(common_bus, routes_by_heater_id, heater_id)
        )
        # A straight drop to the stripe first (the branch a lone heater is
        # meant to have); the greedy search only when no drop column is legal.
        new_path = straight_drop_to_bus(
            individual_terminal,
            obstacle_map,
            config,
            tree_cells=tree_without_own,
            blocked=set(obstacle_map.blocked_cells),
        )
        if new_path is None:
            new_path = shortest_path_to_tree(
                individual_terminal,
                tree_cells=tree_without_own,
                blocked=blocked,
                forbidden=forbidden,
                obstacle_map=obstacle_map,
            )
        if new_path is None:
            continue

        new_selected[heater_id] = individual_terminal
        new_unselected[heater_id] = bus_terminal
        routes_by_heater_id[heater_id] = TerminalBusRoute(
            heater_id=heater_id,
            terminal=individual_terminal,
            path=new_path,
            cost=max(0, len(new_path) - 1),
            access_anchor_cell=terminal_access_anchor_cell(obstacle_map, individual_terminal.id),
            route_start_cell=new_path[0] if new_path else None,
            used_access_anchor=route_uses_access_anchor(
                obstacle_map, individual_terminal.id, new_path
            ),
        )
        swapped_heater_ids.append(heater_id)

    if not swapped_heater_ids:
        return None

    new_tree_cells = set(common_bus.bus.cells)
    for route in routes_by_heater_id.values():
        new_tree_cells.update(route.path)

    new_common_bus = replace(
        common_bus,
        selected_terminals=new_selected,
        unselected_terminals=new_unselected,
        routes=tuple(routes_by_heater_id[route.heater_id] for route in common_bus.routes),
        tree_cells=frozenset(new_tree_cells),
    )
    return new_common_bus, tuple(swapped_heater_ids)


def _bus_tree_cells_excluding(
    common_bus: CommonBusRoutingResult,
    routes_by_heater_id: dict[str, TerminalBusRoute],
    excluded_heater_id: str,
) -> set[GridCell]:
    """Return the bus stripe plus every branch's cells except one heater's own.

    A swapped terminal searches for the tree without the branch it replaces:
    ``common_bus.tree_cells`` still holds that branch's cells, but only the
    final routes are realized, so a new branch landing on the discarded one
    would be metal that reaches nothing.
    """

    cells = set(common_bus.bus.cells)
    for heater_id, route in routes_by_heater_id.items():
        if heater_id != excluded_heater_id:
            cells.update(route.path)
    return cells


def _bus_column_group_heater_ids(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
) -> set[str]:
    """Return heater ids whose bus terminal shares a grid column with another's."""

    heater_ids_by_column: dict[int, list[str]] = {}
    for route in common_bus.routes:
        access = obstacle_map.common_bus_port_accesses.get(route.terminal.id)
        column = access.anchor_cell[0] if access is not None else None
        if column is None and route.path:
            column = route.path[0][0]
        if column is None:
            continue
        heater_ids_by_column.setdefault(column, []).append(route.heater_id)
    return {
        heater_id
        for heater_ids in heater_ids_by_column.values()
        if len(heater_ids) > 1
        for heater_id in heater_ids
    }
