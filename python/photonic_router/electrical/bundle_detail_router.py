"""Detailed centerline routing for topology-derived individual bundles.

Collects every escape bundle's members, channel-routes them jointly
(``river_routing.route_bundles_as_channels``: corridor lanes per bundle, one
global track assignment), commits every placed wire in track order, and
falls the rest -- wire by wire, not bundle by bundle -- back to the
per-wire full-grid search (``pad_wire_search.route_full_grid_pad_wire``).
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Literal

from photonic_router.electrical.individual_topology import bundle_route_side
from photonic_router.electrical.metal_realization import common_bus_escape_rects
from photonic_router.electrical.pad_slots import pad_access_cells
from photonic_router.electrical.pad_wire_search import route_full_grid_pad_wire
from photonic_router.electrical.pitch_grid import bbox_to_grid_cells
from photonic_router.electrical.river_routing import (
    Member,
    PadWireBlockers,
    count_direction_changes,
    route_bundles_as_channels,
)
from photonic_router.electrical.terminal_contacts import orientation_distance
from photonic_router.electrical.types import (
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
from photonic_router.electrical.wire_geometry import (
    cells_from_point_path,
    individual_terminal_open_cells,
    pad_wire_terminal_open_cells,
    terminal_exit_direction_x,
    wire_reservation_cells_from_point_path,
    wire_spacing_radius_cells,
)
from photonic_router.records import fallbacks

RouteSide = Literal["left", "right"]


@dataclass
class _DetailProgress:
    """What the detail router has placed and refused so far.

    ``committed_cells`` are the cells of the placed wires' paths and
    ``committed_footprint_cells`` those cells widened by the wire spacing
    radius, which a later wire's bare centerline must not touch.
    ``cell_usage`` counts the placed wires per path cell.
    """

    obstacle_map: ElectricalObstacleMap
    config: ElectricalRoutingConfig
    routes: list[DetailedBundleRoute] = field(default_factory=list)
    failed_routes: list[DetailedBundleRoute] = field(default_factory=list)
    cell_usage: dict[GridCell, int] = field(default_factory=dict)
    committed_cells: set[GridCell] = field(default_factory=set)
    committed_footprint_cells: set[GridCell] = field(default_factory=set)

    def commit(
        self,
        detailed_path: tuple[tuple[float, float], ...],
        member: Member,
        *,
        bundle_id: int,
        shape: Literal["L", "Z"],
        contact_port_name: str | None,
        construction: Literal["river", "fallback"],
    ) -> None:
        """Record a placed wire and reserve its cells and its spacing footprint."""

        rank, terminal, assignment, topology_route, target_cells = member
        path = cells_from_point_path(detailed_path)
        route = DetailedBundleRoute(
            bundle_id=bundle_id,
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
        self.routes.append(route)
        self.committed_cells.update(path)
        self.committed_footprint_cells.update(
            wire_reservation_cells_from_point_path(
                detailed_path,
                self.obstacle_map,
                radius=wire_spacing_radius_cells(self.obstacle_map, self.config),
            )
        )
        for cell in path:
            self.cell_usage[cell] = self.cell_usage.get(cell, 0) + 1

    def fail(
        self,
        *,
        bundle_id: int,
        rank: int,
        terminal: ElectricalTerminal,
        assignment: PadAssignment | None,
        target_cells: frozenset[GridCell],
        reason: str,
    ) -> None:
        """Record a wire that could not be placed, with the reason."""

        self.failed_routes.append(
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


def _pad_wire_blockers(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> PadWireBlockers:
    """The cells closed to the pad wires before the first one is placed.

    Closed to every wire: the obstacle map's blocked cells, the bus tree,
    the bus escape's path and its realized rectangles. Closed to every wire
    but the owner: the individual terminals' open cells and the pads.
    """

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
    assigned_pad_cells_by_slot = {
        assignment.slot.index: bbox_to_grid_cells(assignment.slot.bbox, obstacle_map.grid)
        for assignment in pad_plan.assignments
    }
    all_assigned_pad_cells = (
        set().union(*assigned_pad_cells_by_slot.values()) if assigned_pad_cells_by_slot else set()
    )
    return PadWireBlockers(
        hard=hard_blocked,
        terminal_cells=all_terminal_cells,
        pad_cells=all_assigned_pad_cells,
        pad_cells_by_slot=assigned_pad_cells_by_slot,
    )


def _collect_bundle_entries(
    topology: IndividualEscapeTopologyResult,
    pad_plan: PadPlan,
    target_cells_by_slot: dict[int, frozenset[GridCell]],
    blockers: PadWireBlockers,
    progress: _DetailProgress,
) -> tuple[list[tuple[int, list[Member]]], dict[str, int]]:
    """Pass 1: the members of every bundle that can be routed, and each terminal's bundle id.

    One entry ``(exit_dx, members)`` per bundle, bundles in pad-row order
    (``topology.bundles`` is already sorted by exit position along the pad
    row). A terminal that fails ``_detail_failure_reason`` is recorded as
    failed and left out.
    """

    obstacle_map = progress.obstacle_map
    assignments_by_terminal_id = {
        assignment.terminal.id: assignment
        for assignment in pad_plan.assignments
        if assignment.kind == "individual" and assignment.terminal is not None
    }
    topology_route_by_terminal_id = {
        route.terminal.id: route for route in topology.routes if route.success
    }
    bundle_entries: list[tuple[int, list[Member]]] = []
    bundle_id_by_terminal_id: dict[str, int] = {}
    for bundle in topology.bundles:
        route_side = bundle_route_side(bundle, obstacle_map)
        exit_dx = 1 if route_side == "right" else -1
        members: list[Member] = []
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
                blockers.hard,
                blockers.terminal_cells,
            )
            if failure_reason is not None:
                progress.fail(
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
            bundle_id_by_terminal_id[terminal.id] = bundle.bundle_id

        bundle_entries.append((exit_dx, members))
    return bundle_entries, bundle_id_by_terminal_id


def _commit_placed_wires(
    placed: Iterable[tuple[Member, tuple[GridPoint, ...], Literal["L", "Z"]]],
    bundle_id_by_terminal_id: dict[str, int],
    progress: _DetailProgress,
) -> list[tuple[Member, str]]:
    """Commit the channel router's wires in their order; return those that must fall back.

    A wire falls back when its bare centerline touches the footprint of a
    wire committed before it.
    """

    to_fall_back: list[tuple[Member, str]] = []
    for member, centerline, shape in placed:
        terminal = member[1]
        # committed_footprint_cells already carries the spacing-radius
        # dilation (see _DetailProgress.commit), so it is tested against this
        # wire's bare centerline, not a re-dilated one -- dilating twice
        # would hold two wires apart by twice their required clearance.
        # Placed wires are committed in track order (lowest first), so a
        # later wire in this same list sees every earlier one's footprint.
        if set(cells_from_point_path(centerline)) & progress.committed_footprint_cells:
            to_fall_back.append(
                (member, f"{terminal.id}'s channel wire is too close to a committed wire")
            )
            continue
        terminal_access = progress.obstacle_map.individual_port_accesses[terminal.id]
        progress.commit(
            centerline,
            member,
            bundle_id=bundle_id_by_terminal_id[terminal.id],
            shape=shape,
            contact_port_name=_pad_wire_contact_port_name(terminal, terminal_access, centerline),
            construction="river",
        )
    return to_fall_back


def _route_fallback_wires(
    to_fall_back: list[tuple[Member, str]],
    bundle_id_by_terminal_id: dict[str, int],
    blockers: PadWireBlockers,
    progress: _DetailProgress,
) -> dict[int, list[str]]:
    """Pass 2: route every wire of ``to_fall_back`` with the full-grid search.

    Returns the reasons the wires fell back, per bundle id, each reason
    once and in the order met. A wire the search cannot connect, or whose
    path touches a committed footprint, is recorded as failed.
    """

    obstacle_map = progress.obstacle_map
    fallback_reasons_by_bundle_id: dict[int, list[str]] = {}
    for member, reason in to_fall_back:
        fallbacks.take("electrical.pad_wire_full_grid")
        rank, terminal, assignment, _topology_route, target_cells = member
        bundle_id = bundle_id_by_terminal_id[terminal.id]
        reasons = fallback_reasons_by_bundle_id.setdefault(bundle_id, [])
        if reason not in reasons:
            reasons.append(reason)
        fallback_access = obstacle_map.individual_port_accesses.get(terminal.id)
        raw_fallback_blocked = blockers.for_member(member, obstacle_map)
        detailed_path = (
            route_full_grid_pad_wire(
                fallback_access,
                target_cells,
                obstacle_map,
                raw_fallback_blocked,
                progress.committed_footprint_cells,
                progress.config,
            )
            if fallback_access is not None
            else ()
        )
        if not detailed_path:
            progress.fail(
                bundle_id=bundle_id,
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
        # cells already carries the spacing-radius dilation (see
        # _DetailProgress.commit), so it is tested against the fallback
        # path's bare centerline, not a re-dilated one -- dilating twice
        # would hold two wires apart by twice their required clearance.
        fallback_bare_cells = set(cells_from_point_path(detailed_path))
        if fallback_bare_cells & progress.committed_footprint_cells:
            progress.fail(
                bundle_id=bundle_id,
                rank=rank,
                terminal=terminal,
                assignment=assignment,
                target_cells=target_cells,
                reason="Z fallback route crosses a sibling's committed metal footprint",
            )
            continue
        assert fallback_access is not None
        fallback_bends = count_direction_changes(detailed_path)
        progress.commit(
            detailed_path,
            member,
            bundle_id=bundle_id,
            shape="L" if fallback_bends <= 1 else "Z",
            contact_port_name=_pad_wire_contact_port_name(terminal, fallback_access, detailed_path),
            construction="fallback",
        )
    return fallback_reasons_by_bundle_id


def _fail_terminals_outside_every_bundle(
    pad_plan: PadPlan,
    target_cells_by_slot: dict[int, frozenset[GridCell]],
    progress: _DetailProgress,
) -> None:
    """Record as failed every individual pad assignment whose terminal no bundle held."""

    routed_terminal_ids = {route.terminal.id for route in progress.routes}
    routed_terminal_ids.update(route.terminal.id for route in progress.failed_routes)
    for assignment in sorted(
        (
            assignment
            for assignment in pad_plan.assignments
            if assignment.kind == "individual" and assignment.terminal is not None
        ),
        key=lambda assignment: assignment.slot.index,
    ):
        unrouted_terminal = assignment.terminal
        if unrouted_terminal is None or unrouted_terminal.id in routed_terminal_ids:
            continue
        progress.failed_routes.append(
            DetailedBundleRoute(
                bundle_id=assignment.topology_bundle_id
                if assignment.topology_bundle_id is not None
                else -1,
                rank=assignment.topology_rank if assignment.topology_rank is not None else -1,
                terminal=unrouted_terminal,
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


def route_detailed_bundles(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult,
    topology: IndividualEscapeTopologyResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> DetailedBundleRoutingResult:
    """Route every individual pad wire as a two-segment L, or a Z fallback.

    Two passes: pass 1 collects every bundle's members (unchanged failure
    checks), then channel-routes every survivor jointly
    (``river_routing.route_bundles_as_channels``: corridor lanes per bundle,
    one global track assignment over every bundle's wires), committing each
    placed wire's reservation footprint before the next, lowest track first.
    Pass 2 gives every wire pass 1 could not place (a bundle refused at the
    corridor-lane step, a track-assignment cycle, insufficient track
    headroom, or a collision) a full-grid A* fallback
    (``pad_wire_search.route_full_grid_pad_wire``) that must avoid every
    placed wire's committed footprint, so a fallback wire can never cross a
    channel-routed one.
    """

    config.validate()
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    track_pitch_cells = max(1, math.ceil(track_pitch_um / obstacle_map.grid.grid_size_um))
    target_cells_by_slot = {
        assignment.slot.index: pad_access_cells(assignment.slot, obstacle_map, config)
        for assignment in pad_plan.assignments
    }
    blockers = _pad_wire_blockers(obstacle_map, common_bus, common_bus_escape, pad_plan, config)
    progress = _DetailProgress(obstacle_map, config)

    # Every bundle's members are collected first; the survivors are
    # channel-routed jointly (corridor lanes per bundle, one global track
    # assignment), and every wire the channel router could not place falls
    # back individually, not bundle by bundle.
    bundle_entries, bundle_id_by_terminal_id = _collect_bundle_entries(
        topology, pad_plan, target_cells_by_slot, blockers, progress
    )
    channel_result = route_bundles_as_channels(bundle_entries, obstacle_map, config, blockers)
    to_fall_back = _commit_placed_wires(channel_result.placed, bundle_id_by_terminal_id, progress)
    to_fall_back.extend(channel_result.unplaced)
    fallback_reasons_by_bundle_id = _route_fallback_wires(
        to_fall_back, bundle_id_by_terminal_id, blockers, progress
    )
    _fail_terminals_outside_every_bundle(pad_plan, target_cells_by_slot, progress)

    return DetailedBundleRoutingResult(
        routes=tuple(sorted(progress.routes, key=_detailed_route_sort_key)),
        failed_routes=tuple(sorted(progress.failed_routes, key=_detailed_route_sort_key)),
        committed_cells=frozenset(progress.committed_cells),
        cell_usage=progress.cell_usage,
        track_pitch_cells=track_pitch_cells,
        bundle_fallback_reasons={
            bundle_id: "; ".join(reasons)
            for bundle_id, reasons in fallback_reasons_by_bundle_id.items()
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
    source_cells = pad_wire_terminal_open_cells(obstacle_map, terminal.id)
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
            orientation_distance(port.orientation, target_orientation),
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
