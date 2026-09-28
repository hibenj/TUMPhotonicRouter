"""Detailed centerline routing for topology-derived individual bundles.

Dispatches each escape bundle to the river construction
(``river_routing.try_river_route_bundle``) and falls the bundle back to the
per-wire full-grid search (``pad_wire_search.route_full_grid_pad_wire``) when
the river construction refuses it.
"""

from __future__ import annotations

import math
from typing import Literal

from .individual_topology import bundle_route_side
from .metal_realization import common_bus_escape_rects
from .pad_slots import pad_access_cells
from .pad_wire_search import route_full_grid_pad_wire
from .pitch_grid import bbox_to_grid_cells
from .river_routing import count_direction_changes, try_river_route_bundle
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
from .wire_geometry import (
    cells_from_point_path,
    individual_terminal_open_cells,
    terminal_exit_direction_x,
    terminal_open_cells,
    wire_reservation_cells_from_point_path,
    wire_spacing_radius_cells,
)

RouteSide = Literal["left", "right"]


def route_detailed_bundles(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult,
    topology: IndividualEscapeTopologyResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> DetailedBundleRoutingResult:
    """Route every individual pad wire as a two-segment L, or a Z fallback.

    Two passes: pass 1 tries every wire as an L (river-routed as a whole
    bundle, see ``river_routing.try_river_route_bundle``), bundles in
    pad-row order and, inside a bundle, the terminal nearest the pad row
    first, committing each success's reservation footprint before the next
    attempt. Pass 2 gives every terminal pass 1 refused (wrong side or a
    blocked column) a full-grid A* fallback
    (``pad_wire_search.route_full_grid_pad_wire``) that must avoid every L's
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
        set().union(*individual_terminal_open_cells(obstacle_map).values())
        if individual_terminal_open_cells(obstacle_map)
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
        path = cells_from_point_path(detailed_path)
        route = DetailedBundleRoute(
            bundle_id=bundle.bundle_id,
            rank=rank,
            terminal=terminal,
            pad_assignment=assignment,
            path=path,
            target_cells=frozenset(target_cells),
            centerline=detailed_path,
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
            wire_reservation_cells_from_point_path(
                detailed_path,
                obstacle_map,
                config,
                radius=wire_spacing_radius_cells(obstacle_map, config),
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
                centerline=(),
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

        river_wires, river_failure_reason = try_river_route_bundle(
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
            source_cells = terminal_open_cells(obstacle_map, terminal.id)
            own_pad_cells = assigned_pad_cells_by_slot.get(assignment.slot.index, target_cells)
            raw_fallback_blocked = set(hard_blocked)
            raw_fallback_blocked.update(all_terminal_cells.difference(source_cells))
            raw_fallback_blocked.update(all_assigned_pad_cells.difference(own_pad_cells))
            detailed_path = (
                route_full_grid_pad_wire(
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
            fallback_bare_cells = set(cells_from_point_path(detailed_path))
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
            fallback_bends = count_direction_changes(detailed_path)
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
                centerline=(),
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
    source_cells = terminal_open_cells(obstacle_map, terminal.id)
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
        return _exit_facing_port_name(terminal, terminal_exit_direction_x(terminal_access))
    return terminal_access.port_name
