"""Orchestrate the first electrical heater-routing milestones."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from .bundle_detail_router import (
    _dilate_cells,
    _wire_reservation_radius_cells,
    route_detailed_bundles,
)
from .common_bus_router import (
    _all_terminal_cells,
    _forbidden_terminal_cells,
    _route_uses_access_anchor,
    _shortest_path_to_tree,
    _terminal_access_anchor_cell,
    route_common_bus,
    straight_drop_to_bus,
)
from .debug import export_electrical_debug_svg, export_electrical_metal_snapshot_svg
from .escape_router import route_common_bus_escape
from .individual_topology import compute_individual_escape_topology
from .metal_realization import realize_electrical_metal
from .obstacle_extraction import build_electrical_obstacle_map
from .pad_slots import plan_pad_slots
from .pitch_grid import bbox_to_grid_cells
from .terminal_extraction import extract_heater_terminal_pairs
from .types import (
    BusStripe,
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
    GridCell,
    PadPlan,
    TerminalBusRoute,
)
from .verification import verify_electrical_routing


def route_electrical_heaters(
    component: Component,
    schematic: Schematic | None = None,
    config: ElectricalRoutingConfig | None = None,
    *,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "electrical",
) -> ElectricalRoutingResult:
    """Run the current electrical heater-routing milestones.

    It extracts heater terminal pairs, builds the electrical obstacle grid,
    routes one terminal per heater to the derived opposite-side common bus,
    plans abstract pad slots, routes the remaining individual terminals to
    those slots, realizes metal polygons, and optionally writes a debug SVG.
    """

    config = config or ElectricalRoutingConfig()
    config.validate()

    terminal_groups = extract_heater_terminal_pairs(component, schematic, config)
    obstacle_map = build_electrical_obstacle_map(component, terminal_groups, config)
    if not terminal_groups:
        routed_component = component.copy()
        return ElectricalRoutingResult(
            terminal_groups=terminal_groups,
            obstacle_map=obstacle_map,
            common_bus=CommonBusRoutingResult(
                bus_side=config.bus_side,
                bus=obstacle_map.bus,
                selected_terminals={},
                unselected_terminals={},
                routes=(),
                tree_cells=frozenset(),
                failed_heaters=(),
            ),
            routed_component=routed_component,
        )

    common_bus = route_common_bus(terminal_groups, obstacle_map, config)
    obstacle_map, common_bus = _trim_common_bus_to_connections(
        obstacle_map,
        common_bus,
        config,
    )
    individual_topology = (
        compute_individual_escape_topology(obstacle_map, common_bus, config)
        if common_bus.success
        else None
    )
    pad_plan = (
        plan_pad_slots(common_bus, obstacle_map, config, individual_topology)
        if common_bus.success
        else None
    )
    if common_bus.success and individual_topology is not None and pad_plan is not None:
        swapped_common_bus = _apply_pad_side_consistency_swap(
            obstacle_map,
            common_bus,
            pad_plan,
            config,
        )
        if swapped_common_bus is not None:
            common_bus = swapped_common_bus
            obstacle_map, common_bus = _trim_common_bus_to_connections(
                obstacle_map,
                common_bus,
                config,
            )
            individual_topology = compute_individual_escape_topology(
                obstacle_map, common_bus, config
            )
            pad_plan = plan_pad_slots(common_bus, obstacle_map, config, individual_topology)
    common_bus_escape = (
        route_common_bus_escape(obstacle_map, common_bus, pad_plan, config)
        if pad_plan is not None
        else None
    )
    detailed_bundle_routes = (
        route_detailed_bundles(
            obstacle_map,
            common_bus,
            common_bus_escape,
            individual_topology,
            pad_plan,
            config,
        )
        if (
            pad_plan is not None
            and common_bus_escape is not None
            and individual_topology is not None
            and common_bus_escape.success
        )
        else None
    )
    routed_component = (
        realize_electrical_metal(
            component,
            obstacle_map,
            common_bus,
            common_bus_escape,
            detailed_bundle_routes,
            pad_plan,
            config,
        )
        if common_bus.success
        else None
    )
    verification = (
        verify_electrical_routing(
            obstacle_map,
            common_bus,
            common_bus_escape,
            detailed_bundle_routes,
            pad_plan,
            config,
        )
        if routed_component is not None
        else None
    )
    artifacts: dict[str, str] = {}
    if debug_dir is not None:
        debug_path = Path(debug_dir) / "electrical" / f"{debug_prefix}_common_bus.svg"
        export_electrical_debug_svg(
            debug_path,
            obstacle_map,
            terminal_groups,
            common_bus,
            common_bus_escape=common_bus_escape,
            individual_topology=individual_topology,
            detailed_bundle_routes=detailed_bundle_routes,
            pad_plan=pad_plan,
        )
        artifacts["common_bus_svg"] = str(debug_path)
        if routed_component is not None:
            metal_snapshot_path = (
                Path(debug_dir) / "electrical" / f"{debug_prefix}_metal_snapshot.svg"
            )
            export_electrical_metal_snapshot_svg(
                metal_snapshot_path,
                routed_component,
                obstacle_map,
                terminal_groups,
                pad_plan,
                config,
            )
            artifacts["metal_snapshot_svg"] = str(metal_snapshot_path)

    return ElectricalRoutingResult(
        terminal_groups=terminal_groups,
        obstacle_map=obstacle_map,
        common_bus=common_bus,
        pad_plan=pad_plan,
        common_bus_escape=common_bus_escape,
        individual_topology=individual_topology,
        detailed_bundle_routes=detailed_bundle_routes,
        routed_component=routed_component,
        verification=verification,
        debug_artifacts=artifacts,
    )


def _apply_pad_side_consistency_swap(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> CommonBusRoutingResult | None:
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
    assignment if that reroute fails.
    """

    grouped_heater_ids = _bus_column_group_heater_ids(obstacle_map, common_bus)
    pad_column_um_by_terminal_id = {
        assignment.terminal.id: assignment.slot.center[0]
        for assignment in pad_plan.assignments
        if assignment.kind == "individual" and assignment.terminal is not None
    }
    all_terminal_cells = _all_terminal_cells(obstacle_map)
    # Dilated by the branch wire's own reservation radius: _shortest_path_to_tree
    # only checks the bare centerline against `blocked`, and the branch is
    # realized with untrimmed bend corners (trim_route_tail_bends=False), so a
    # bend placed right at an obstacle's edge can still draw metal that clips
    # it. The router's own route_common_bus loop never hit this because no
    # bend it chose ever landed that close; a swap-driven reroute is not
    # guaranteed the same luck.
    reservation_radius = _wire_reservation_radius_cells(obstacle_map, config)
    blocked = _dilate_cells(
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
        exit_dx = _terminal_side_exit_dx(individual_terminal.side_key)
        pad_column_um = pad_column_um_by_terminal_id.get(individual_terminal.id)
        if exit_dx == 0 or pad_column_um is None:
            continue
        if exit_dx * (pad_column_um - individual_terminal.center[0]) >= 0:
            continue  # exit already points toward the pad's column.

        bus_terminal = common_bus.selected_terminals[heater_id]
        forbidden = _forbidden_terminal_cells(
            obstacle_map,
            all_terminal_cells,
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
            new_path = _shortest_path_to_tree(
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
            access_anchor_cell=_terminal_access_anchor_cell(obstacle_map, individual_terminal.id),
            route_start_cell=new_path[0] if new_path else None,
            used_access_anchor=_route_uses_access_anchor(
                obstacle_map, individual_terminal.id, new_path
            ),
        )
        swapped_heater_ids.append(heater_id)

    if not swapped_heater_ids:
        return None

    new_tree_cells = set(common_bus.bus.cells)
    for route in routes_by_heater_id.values():
        new_tree_cells.update(route.path)

    return replace(
        common_bus,
        selected_terminals=new_selected,
        unselected_terminals=new_unselected,
        routes=tuple(routes_by_heater_id[route.heater_id] for route in common_bus.routes),
        tree_cells=frozenset(new_tree_cells),
    )


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


def _terminal_side_exit_dx(side_key: str) -> int:
    """Return -1/+1 for a terminal's ``l``/``r`` exit side, 0 if unknown."""

    if side_key == "l":
        return -1
    if side_key == "r":
        return 1
    return 0


def _trim_common_bus_to_connections(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    config: ElectricalRoutingConfig,
) -> tuple[ElectricalObstacleMap, CommonBusRoutingResult]:
    """Limit realized/debug bus stripe to the span touched by routed terminals."""

    if not common_bus.routes:
        return obstacle_map, common_bus

    provisional_bus_cells = obstacle_map.bus.cells
    connection_cells = {
        cell for route in common_bus.routes for cell in route.path if cell in provisional_bus_cells
    }
    if not connection_cells:
        return obstacle_map, common_bus

    min_x_um = min(_grid_cell_center_um(cell, obstacle_map)[0] for cell in connection_cells)
    max_x_um = max(_grid_cell_center_um(cell, obstacle_map)[0] for cell in connection_cells)
    half_overlap_um = max(
        obstacle_map.grid.grid_size_um / 2.0,
        config.wire_width_um / 2.0,
    )
    _, bus_ymin, _, bus_ymax = obstacle_map.bus.bbox
    trimmed_bbox = (
        min_x_um - half_overlap_um,
        bus_ymin,
        max_x_um + half_overlap_um,
        bus_ymax,
    )
    trimmed_bus_cells = bbox_to_grid_cells(trimmed_bbox, obstacle_map.grid)
    trimmed_bus = BusStripe(
        side=obstacle_map.bus.side,
        bbox=trimmed_bbox,
        cells=frozenset(trimmed_bus_cells),
    )
    trimmed_tree_cells = set(common_bus.tree_cells).difference(provisional_bus_cells) | set(
        trimmed_bus_cells
    )
    trimmed_obstacle_map = replace(obstacle_map, bus=trimmed_bus)
    trimmed_common_bus = replace(
        common_bus,
        bus=trimmed_bus,
        tree_cells=frozenset(trimmed_tree_cells),
    )
    return trimmed_obstacle_map, trimmed_common_bus


def _grid_cell_center_um(
    cell: GridCell,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[float, float]:
    x, y = cell
    grid = obstacle_map.grid
    origin_x, origin_y = grid.origin
    return (
        origin_x + (x + 0.5) * grid.grid_size_um,
        origin_y + (y + 0.5) * grid.grid_size_um,
    )
