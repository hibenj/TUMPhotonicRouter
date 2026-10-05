"""Tests for translation/electrical/bundle_detail_router.py, river_routing.py
and pad_wire_search.py.

Stage 9 (``PadWireRouter``): routing every individual pad wire as an L, or a
Z fallback -- the dispatcher and the channel-routing/search construction
under it. ``assign_tracks`` (the global track assignment, 2026-09-29 channel
routing plan) is pure and tested directly on synthetic wire sets below,
independent of any escape bundle or obstacle map.
"""

from __future__ import annotations

import pytest

from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.pad_wire_search import _pad_stub_step_cost
from translation.electrical.river_routing import ChannelWire, assign_tracks
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


# --- assign_tracks: pure unit tests on synthetic wire sets (channel routing
# plan, 2026-09-29). ``assign_tracks`` never reads an obstacle map or a
# config; every wire is a bare (id, corridor lane c, pad column p, row)
# tuple, so these are independent of any bundle, benchmark or grid.


def test_assign_tracks_places_one_outward_nested_bundle_in_row_order():
    """A four-row outward-nested bundle: corridor lanes cluster near the
    exit column (c increasing with row), pad columns cluster near the pad
    row (p increasing with row, all p > all c) -- row 1's pad column falls
    inside every other row's span (each row's own corridor lane is closer to
    the exit, so a later row's span always reaches back across row 1's pad
    column too), so row 1 must lie above every other row, and so on down
    the stack: four tracks, row 1 on the highest (nearest the pads).
    """

    wires = [
        ChannelWire("row1", c=10, p=50, row=1),
        ChannelWire("row2", c=20, p=60, row=2),
        ChannelWire("row3", c=30, p=70, row=3),
        ChannelWire("row4", c=40, p=80, row=4),
    ]

    placed, unplaced = assign_tracks(wires, track_pitch_cells=5)

    assert unplaced == []
    assert placed == {"row1": 4, "row2": 3, "row3": 2, "row4": 1}


def test_assign_tracks_three_wire_case_from_cascade_16_folded():
    """The measured case that motivated this plan (owner's observation,
    2026-09-29): W's corridor lane and pad column at cell x 1854 and 2334,
    V's at 2260 and 2464, U's at 2290 and 2594 -- W's own pad column (2334)
    falls inside both V's and U's spans, and V's pad column (2464) falls
    inside U's span, so W must sit above V and U, and V above U: three
    tracks, W nearest the pads, U farthest.
    """

    wires = [
        ChannelWire("W", c=1854, p=2334, row=1),
        ChannelWire("V", c=2260, p=2464, row=1),
        ChannelWire("U", c=2290, p=2594, row=2),
    ]

    placed, unplaced = assign_tracks(wires, track_pitch_cells=10)

    assert unplaced == []
    assert placed == {"W": 3, "V": 2, "U": 1}


def test_assign_tracks_reports_a_two_wire_cycle_as_unplaced():
    """W's corridor lane falls inside R's span (R must sit above W) while
    R's corridor lane also falls inside W's span (W must sit above R): no
    consistent order exists, so both wires are returned unplaced for the
    per-wire fallback.
    """

    wires = [
        ChannelWire("W", c=10, p=30, row=1),
        ChannelWire("R", c=20, p=5, row=1),
    ]

    placed, unplaced = assign_tracks(wires, track_pitch_cells=3)

    assert placed == {}
    assert sorted(unplaced) == ["R", "W"]


def test_assign_tracks_packs_disjoint_spans_onto_one_track():
    """Two wires whose spans are far apart (well beyond one pitch) share
    the lowest track: neither constrains the other, and the left-edge scan
    places both in one pass.
    """

    wires = [
        ChannelWire("A", c=10, p=20, row=1),
        ChannelWire("B", c=100, p=110, row=1),
    ]

    placed, unplaced = assign_tracks(wires, track_pitch_cells=5)

    assert unplaced == []
    assert placed == {"A": 1, "B": 1}


def test_assign_tracks_fills_from_the_top_so_an_unconstrained_wire_stays_near_its_pad():
    """Second slice (2026-09-29): a lone wire far from a three-row nested
    bundle has no constraint at all. Filling the tracks from the bottom put
    it on track 1, the track farthest from the pads, next to the layout's
    edge; filling from the top with row preference puts it on the highest
    track, beside the bundle's own row 1, where its run is nearest its pad.
    """

    wires = [
        ChannelWire("row1", c=10, p=50, row=1),
        ChannelWire("row2", c=20, p=60, row=2),
        ChannelWire("row3", c=30, p=70, row=3),
        ChannelWire("lone", c=200, p=220, row=1),
    ]

    placed, unplaced = assign_tracks(wires, track_pitch_cells=5)

    assert unplaced == []
    assert placed == {"row1": 3, "row2": 2, "row3": 1, "lone": 3}


def test_assign_tracks_holds_a_pad_column_within_a_pitch_of_a_lane_as_a_crossing():
    """The measured cascade_12_folded pair (second slice, 2026-09-29): a
    bundle's outermost row ends at pad column 683, a lone heater's corridor
    lane starts at 681 -- two cells apart at a three-cell pitch. Neither x
    falls inside the other's span literally, yet the dilated wires collide,
    so the margin of one pitch less one cell counts it as a crossing: the
    lone heater must lie below the bundle's row, two tracks, not one.
    """

    wires = [
        ChannelWire("heater_post_5", c=630, p=683, row=6),
        ChannelWire("heater_extra_0", c=681, p=696, row=1),
    ]

    placed, unplaced = assign_tracks(wires, track_pitch_cells=3)

    assert unplaced == []
    assert placed == {"heater_post_5": 2, "heater_extra_0": 1}


# Also marked e2e (routes a full 16-mode cascade benchmark end to end,
# ~9 s): keeps the fast "-m not e2e" suite under its enforced 60 s budget
# (scripts/test_baseline.sh).
@pytest.mark.integration
@pytest.mark.e2e
def test_cascade_16_folded_routes_its_lone_heater_and_neighbour_bundle_clean(
    electrical_benchmark_result,
):
    """The bug this plan fixes (2026-09-29 channel routing plan, Purpose):
    on ``cascade_16_folded``, ``heater_extra_0``'s lone-heater bundle exits
    under a neighbouring bundle's own corridor/pad run; the old per-bundle
    nesting construction refused both bundles and the fallback for
    ``heater_extra_2:l`` could not find a path at all. The joint track
    assignment gives the three wires distinct tracks instead: all three
    river-route, and the benchmark verifies electrically clean.
    """

    result = electrical_benchmark_result("cascade_16_folded")

    assert result.verification is not None
    assert result.verification.success, result.verification.issues

    routes_by_terminal_id = {
        route.terminal.id: route for route in result.detailed_bundle_routes.routes
    }
    for terminal_id in ("heater_extra_0:r", "heater_extra_1:l", "heater_extra_2:l"):
        route = routes_by_terminal_id[terminal_id]
        assert route.construction == "river", (terminal_id, route.construction)


# Also marked e2e (routes a full 8-mode cascade-equivalent benchmark end to
# end): keeps the fast "-m not e2e" suite under its enforced 60 s budget
# (scripts/test_baseline.sh).
@pytest.mark.integration
@pytest.mark.e2e
def test_heater_s_mod_every_wire_river_routes_with_zero_detour(electrical_benchmark_result):
    """``heater_s_mod``'s bundles nest cleanly under the joint track
    assignment (its old per-bundle "inward"/"outward" mirrored nesting was a
    special case of this, not a separate rule): every wire is "river" and
    every wire's detour is 0 (each one is a monotone L or a single-bend-pair
    Z through its own shared track, never a full-grid search).
    """

    result = electrical_benchmark_result("heater_s_mod")

    assert result.verification is not None
    assert result.verification.success, result.verification.issues
    detailed = result.detailed_bundle_routes
    assert detailed.failed_routes == ()
    assert detailed.bundle_fallback_reasons == {}
    assert all(route.construction == "river" for route in detailed.routes)
    assert result.verification.metrics["pad_wire_detour_total_um"] == 0.0
