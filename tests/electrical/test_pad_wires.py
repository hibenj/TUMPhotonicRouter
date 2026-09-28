"""Tests for translation/electrical/bundle_detail_router.py, river_routing.py
and pad_wire_search.py.

Stage 9 (``PadWireRouter``): routing every individual pad wire as an L, or a
Z fallback -- the dispatcher and the river/search construction under it.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from tests.electrical.support import _terminal
from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.pad_wire_search import _pad_stub_step_cost
from translation.electrical.river_routing import try_river_route_bundle
from translation.electrical.types import (
    BusStripe,
    ElectricalObstacleMap,
    ElectricalPortAccess,
    EscapeTopologyRoute,
    PadAssignment,
    PadSlot,
)
from translation.layout_from_schematic import layout_from_schematic


def test_detailed_bundle_router_nests_l_and_z_pad_wires_from_topology():
    """Milestone 2/4: pad wires are an L or a full-grid Z fallback.

    Superseded the old "spaced lane offsets" expectation: pad wires no
    longer carry a per-rank lane offset (every wire is its own single
    centerline, ``DetailedBundleRoute.centerline``), and a bundle's ranks
    are nested directly into pad columns instead. This
    exercises a config the milestone-2/4 four benchmark fixtures don't
    (a coarser routing grid, wider pad pitch, tight clearance margin), which
    congests enough that some of this schematic's wires need -- and some do
    not find -- a Z fallback; the four benchmark fixtures are covered by
    the fixture-specific tests below instead, so this only asserts the
    routed members of the first two bundles nest correctly and that
    every route that *did* succeed, anywhere in the schematic, is
    geometrically sound.
    """

    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=10.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.detailed_bundle_routes is not None
    detailed = result.detailed_bundle_routes
    assert detailed.track_pitch_cells == 2

    first_bundle_routes = sorted(
        (
            route
            for route in detailed.routes
            if route.bundle_id == result.individual_topology.bundles[0].bundle_id
        ),
        key=lambda route: route.rank,
    )
    assert first_bundle_routes
    ordered_first_terminal_ids = [
        terminal.id for terminal in result.individual_topology.bundles[0].ordered_terminals
    ]
    assert [route.terminal.id for route in first_bundle_routes] == sorted(
        (route.terminal.id for route in first_bundle_routes),
        key=ordered_first_terminal_ids.index,
    )
    assert [route.pad_assignment.slot.index for route in first_bundle_routes] == sorted(
        route.pad_assignment.slot.index for route in first_bundle_routes
    )
    right_bundle_routes = sorted(
        (
            route
            for route in detailed.routes
            if route.bundle_id == result.individual_topology.bundles[1].bundle_id
        ),
        key=lambda route: route.rank,
    )
    # This bundle's outer members compete for the same narrow detour past a
    # neighboring heater at this config's coarse grid and wide pad pitch, so
    # not all four necessarily find a route; the ones that do must still be
    # real members of the bundle, nested in rank order.
    assert right_bundle_routes
    ordered_right_terminal_ids = [
        terminal.id for terminal in result.individual_topology.bundles[1].ordered_terminals
    ]
    assert [route.terminal.id for route in right_bundle_routes] == sorted(
        (route.terminal.id for route in right_bundle_routes),
        key=ordered_right_terminal_ids.index,
    )
    assert [route.pad_assignment.slot.index for route in right_bundle_routes] == sorted(
        route.pad_assignment.slot.index for route in right_bundle_routes
    )

    for route in detailed.routes:
        source_cells = result.obstacle_map.individual_terminal_open_cells[route.terminal.id]
        other_terminal_cells = set().union(
            *(
                cells
                for terminal_id, cells in result.obstacle_map.individual_terminal_open_cells.items()
                if terminal_id != route.terminal.id
            )
        )
        assert route.path[0] in source_cells
        assert route.path[-1] in route.target_cells
        # A pad wire (L or Z) is now always its own single centerline: no
        # separate lane-track leg exists any more.
        assert route.centerline
        assert not set(route.path).intersection(other_terminal_cells)
        offset_cells = {(round(x - 0.5), round(y - 0.5)) for x, y in route.centerline}
        hard_blocked = set(result.obstacle_map.blocked_cells)
        hard_blocked.update(result.common_bus.tree_cells)
        hard_blocked.update(result.common_bus_escape.path)
        assert not set(route.path).intersection(hard_blocked)
        assert not offset_cells.intersection(hard_blocked)
        assert not offset_cells.intersection(other_terminal_cells)


def test_pad_stub_cost_penalizes_turns_and_backtracking():
    config = ElectricalRoutingConfig(pad_side="top")
    targets = frozenset({(5, 5)})

    straight_toward_target = _pad_stub_step_cost(
        (3, 5),
        (4, 5),
        0,
        0,
        targets,
        config,
    )
    turning_away_from_target = _pad_stub_step_cost(
        (3, 5),
        (3, 6),
        0,
        2,
        targets,
        config,
    )
    reversing_laterally = _pad_stub_step_cost(
        (3, 5),
        (2, 5),
        0,
        1,
        targets,
        config,
    )
    moving_away_from_pad_edge = _pad_stub_step_cost(
        (3, 4),
        (3, 3),
        2,
        3,
        targets,
        config,
    )

    assert turning_away_from_target > straight_toward_target
    assert reversing_laterally > straight_toward_target
    assert moving_away_from_pad_edge > straight_toward_target


def test_river_serves_an_inward_row_one_pad():
    """Rule A part 2 (eight-row-bundles plan): row 1's pad may be inward of
    its own corridor lane c_1 -- pad_slots anchors row 1 at or beyond c_1
    (Rule A part 1), but the interval push rule can still shove a "left"
    -side bundle's row 1 back inward by up to one pad pitch. The river
    construction absorbs exactly that instead of refusing the bundle: the
    same four-point centerline (out to c_1, up to h_1, across, into the
    pad), which becomes a Z because the crossing run now reverses
    direction back toward the terminal.
    """

    grid = SimpleNamespace(width=100, height=100, grid_size_um=1.0, origin=(0.0, 0.0))
    terminal = _terminal("heater_x:r", (10.5, 10.5))
    access = ElectricalPortAccess(
        terminal_id=terminal.id,
        heater_id=terminal.heater_id,
        purpose="individual",
        side="top",
        port_name="e1",
        port_point_um=(10.5, 10.5),
        port_orientation_deg=0.0,
        port_width_um=4.0,
        contact_bbox=(8.5, 8.5, 12.5, 12.5),
        anchor_cell=(10, 10),
        anchor_point_um=(10.5, 10.5),
        access_centerline_um=(),
        opened_cells=frozenset({(10, 10)}),
        reserved_cells=frozenset(),
        access_length_um=0.0,
    )
    obstacle_map = ElectricalObstacleMap(
        grid=grid,
        raw_blocked_cells=frozenset(),
        blocked_cells=frozenset(),
        terminal_open_cells={terminal.id: frozenset({(10, 10)})},
        bus=BusStripe(side="bottom", bbox=(0.0, 0.0, 1.0, 1.0), cells=frozenset()),
        die_bbox=(0.0, 0.0, 100.0, 100.0),
        layout_bbox=(0.0, 0.0, 100.0, 100.0),
        individual_terminal_open_cells={terminal.id: frozenset({(10, 10)})},
        individual_port_accesses={terminal.id: access},
    )
    config = ElectricalRoutingConfig(
        pad_side="top",
        wire_width_um=2.0,
        individual_route_spacing_um=0.0,
        obstacle_clearance_um=1.0,
        routing_grid_pitch_um=1.0,
    )
    # exit_column = opening_edge(10) + clearance_cells(1) = 11;
    # c_1 = exit_column + track_pitch_cells(2) = 13; the pad column (10) is
    # deliberately left of it (inward, for this exit_dx=+1 bundle).
    pad_slot = PadSlot(index=0, center=(10.5, 50.5), bbox=(9.5, 48.5, 11.5, 52.5), side="top")
    assignment = PadAssignment(
        slot=pad_slot,
        net_id="individual:heater_x",
        kind="individual",
        terminal=terminal,
        heater_id="heater_x",
    )
    topology_route = EscapeTopologyRoute(
        terminal=terminal, path=((10, 10),), exit_cell=(10, 10), success=True
    )
    target_cells = frozenset({(10, 50)})
    members = [(0, terminal, assignment, topology_route, target_cells)]

    wires, reason = try_river_route_bundle(
        members,
        exit_dx=1,
        obstacle_map=obstacle_map,
        config=config,
        hard_blocked=set(),
        all_terminal_cells={(10, 10)},
        all_assigned_pad_cells={(10, 50)},
        assigned_pad_cells_by_slot={0: frozenset({(10, 50)})},
        committed_footprint_cells=set(),
    )

    assert reason is None
    assert wires is not None
    _member, centerline, shape = wires[0]
    assert shape == "Z"
    xs = [point[0] for point in centerline]
    assert xs[1] > xs[0]  # out to c_1 (13.5), away from the terminal (10.5)
    assert xs[-1] < xs[1]  # back inward to the pad column (10.5)


# Also marked e2e (routes a full 16-mode cascade benchmark end to end,
# ~9 s): keeps the fast "-m not e2e" suite under its enforced 60 s budget
# (scripts/test_baseline.sh).
@pytest.mark.integration
@pytest.mark.e2e
def test_eight_row_bundles_are_river_routed_on_cascade_16_folded(electrical_benchmark_result):
    """Rules A and B (eight-row-bundles plan): ``cascade_16_folded`` has four
    eight-terminal escape bundles (``heater_0..7``, ``heater_post_0..7``,
    and two from the "extra" cascade heaters); the two core ones -- the
    eight-row groups the plan's Purpose section names -- river-route
    cleanly with no fallback and no failed route.
    """

    result = electrical_benchmark_result("cascade_16_folded")

    assert result.detailed_bundle_routes is not None
    fallback_reasons = result.detailed_bundle_routes.bundle_fallback_reasons
    bundles_by_id = {bundle.bundle_id: bundle for bundle in result.individual_topology.bundles}
    core_bundle_ids = {
        bundle_id
        for bundle_id, bundle in bundles_by_id.items()
        if {terminal.heater_id for terminal in bundle.ordered_terminals}
        in (
            {f"heater_{i}" for i in range(8)},
            {f"heater_post_{i}" for i in range(8)},
        )
    }
    assert len(core_bundle_ids) == 2
    assert not (core_bundle_ids & fallback_reasons.keys())

    failed_heater_ids = {
        route.terminal.heater_id for route in result.detailed_bundle_routes.failed_routes
    }
    core_heater_ids = {f"heater_{i}" for i in range(8)} | {f"heater_post_{i}" for i in range(8)}
    assert not (failed_heater_ids & core_heater_ids)
