"""Tests for translation/electrical/bundle_detail_router.py, river_routing.py
and pad_wire_search.py.

Stage 9 (``PadWireRouter``): routing every individual pad wire as an L, or a
Z fallback -- the dispatcher and the river/search construction under it.
"""

from __future__ import annotations

from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.pad_wire_search import _pad_stub_step_cost
from translation.layout_from_schematic import layout_from_schematic


def test_detailed_bundle_router_nests_l_and_z_pad_wires_from_topology():
    """Milestone 2/4: pad wires are an L or a full-grid Z fallback.

    Superseded the old "spaced lane offsets" expectation: pad wires no
    longer carry a per-rank lane offset (every wire is its own single
    centerline, ``DetailedBundleRoute.centerline``), and a bundle's ranks
    are nested directly into pad columns instead. This
    exercises a config the milestone-2/4 four benchmark fixtures don't
    (a coarser routing grid, wider pad pitch, zero clearance margin), which
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
        obstacle_clearance_um=0.0,
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
