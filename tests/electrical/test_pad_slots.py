"""Tests for python/photonic_router/electrical/pad_slots.py.

Stage 6 (``PadPlanner``): assigning used electrical nets to legal pad-pitch
slots, both forced and automatic pad-row placement.
"""

from __future__ import annotations

from itertools import pairwise

import pytest
from photonic_router.electrical import ElectricalRoutingConfig, route_electrical_heaters
from photonic_router.electrical.pad_slots import pad_access_bbox
from photonic_router.records.layout_from_schematic import layout_from_schematic

from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from tests.fixtures.synthetic_layouts import (
    ripup_reroute_heater_schematic as build_ripup_reroute_schematic,
)
from tests.fixtures.synthetic_layouts import (
    single_heater_schematic as build_single_heater_schematic,
)


def test_pad_plan_assigns_slots_without_realizing_geometry():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=10.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.pad_plan is not None
    pad_plan = result.pad_plan
    assert pad_plan.side == "top"
    assert pad_plan.origin_x_um == 0.0
    assert [assignment.kind for assignment in pad_plan.assignments] == [
        "individual",
        "common_bus",
    ]
    assert pad_plan.assignments[0].slot.index == 0
    assert pad_plan.common_bus_assignment.slot.index == max(slot.index for slot in pad_plan.slots)
    assert pad_plan.empty_slots
    assert pad_plan.common_bus_assignment is not None
    for assignment in pad_plan.assignments:
        if assignment.kind == "individual":
            assert assignment.slot.center[0] == assignment.slot.index * config.pad_pitch_um
        else:
            xmin, _, xmax, _ = assignment.slot.bbox
            assert xmax - xmin == pytest.approx(config.common_bus_bondpad_width_um)


def test_pad_access_is_only_on_chip_facing_pad_edge():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    top_config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        pad_access_depth_um=20.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=10.0,
        terminal_open_radius_um=20.0,
    )
    bottom_config = ElectricalRoutingConfig(
        pad_side="bottom",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        pad_access_depth_um=20.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=10.0,
        terminal_open_radius_um=20.0,
    )

    top_result = route_electrical_heaters(component, schematic, top_config)
    bottom_result = route_electrical_heaters(component, schematic, bottom_config)

    top_slot = top_result.pad_plan.common_bus_assignment.slot
    bottom_slot = bottom_result.pad_plan.common_bus_assignment.slot
    top_half_width = top_config.wire_width_um / 2.0
    bottom_half_width = bottom_config.wire_width_um / 2.0
    assert top_slot.side == top_config.pad_side
    assert bottom_slot.side == bottom_config.pad_side
    assert pad_access_bbox(top_slot, top_config) == (
        top_slot.center[0] - top_half_width,
        top_slot.bbox[1],
        top_slot.center[0] + top_half_width,
        top_slot.bbox[1] + top_config.pad_access_depth_um,
    )
    assert pad_access_bbox(bottom_slot, bottom_config) == (
        bottom_slot.center[0] - bottom_half_width,
        bottom_slot.bbox[3] - bottom_config.pad_access_depth_um,
        bottom_slot.center[0] + bottom_half_width,
        bottom_slot.bbox[3],
    )
    top_bus_half_width = top_config.bus_width_um / 2.0
    assert pad_access_bbox(
        top_slot,
        top_config,
        width_um=top_config.bus_width_um,
    ) == (
        top_slot.center[0] - top_bus_half_width,
        top_slot.bbox[1],
        top_slot.center[0] + top_bus_half_width,
        top_slot.bbox[1] + top_config.pad_access_depth_um,
    )


def test_top_pad_offset_moves_pad_row_further_from_layout():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    near = route_electrical_heaters(
        component,
        schematic,
        ElectricalRoutingConfig(
            pad_side="top",
            pad_pitch_um=150.0,
            pad_origin_x_um=0.0,
            pad_offset_um=40.0,
            routing_grid_pitch_um=20.0,
            obstacle_clearance_um=10.0,
            terminal_open_radius_um=20.0,
        ),
    )
    far = route_electrical_heaters(
        component,
        schematic,
        ElectricalRoutingConfig(
            pad_side="top",
            pad_pitch_um=150.0,
            pad_origin_x_um=0.0,
            pad_offset_um=200.0,
            routing_grid_pitch_um=20.0,
            obstacle_clearance_um=10.0,
            terminal_open_radius_um=20.0,
        ),
    )

    near_slot = next(
        assignment.slot
        for assignment in near.pad_plan.assignments
        if assignment.kind == "individual"
    )
    far_slot = next(
        assignment.slot
        for assignment in far.pad_plan.assignments
        if assignment.kind == "individual"
    )
    assert far_slot.bbox[1] > near_slot.bbox[1]
    assert far_slot.center[1] > near_slot.center[1]


def test_pad_plan_allows_empty_pitch_slots_and_keeps_individual_order():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        pad_empty_slots_between_assignments=1,
        pad_extra_slots_left=1,
        pad_extra_slots_right=1,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=10.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.pad_plan is not None
    pad_plan = result.pad_plan
    assignments = pad_plan.assignments
    individual_assignments = [
        assignment for assignment in assignments if assignment.kind == "individual"
    ]
    individual_xs = [assignment.terminal.center[0] for assignment in individual_assignments]
    slot_xs = [assignment.slot.center[0] for assignment in individual_assignments]

    assert individual_xs == sorted(individual_xs)
    assert slot_xs == sorted(slot_xs)
    assert pad_plan.common_bus_assignment == assignments[-1]
    assert (
        pad_plan.common_bus_assignment.slot.index == max(slot.index for slot in pad_plan.slots) - 1
    )
    assert pad_plan.empty_slots
    assert 0 in {slot.index for slot in pad_plan.empty_slots}
    assert max(slot.index for slot in pad_plan.slots) in {
        slot.index for slot in pad_plan.empty_slots
    }
    for assignment in assignments:
        if assignment.kind == "individual":
            assert assignment.slot.center[0] == assignment.slot.index * config.pad_pitch_um


def test_auto_pad_origin_compacts_row_toward_escape_topology():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    base_kwargs = {
        "pad_side": "top",
        "pad_pitch_um": 150.0,
        "routing_grid_pitch_um": 20.0,
        "obstacle_clearance_um": 10.0,
        "terminal_open_radius_um": 20.0,
        "wire_width_um": 20.0,
        "individual_route_spacing_um": 20.0,
    }

    forced = route_electrical_heaters(
        component,
        schematic,
        ElectricalRoutingConfig(**base_kwargs, pad_origin_x_um=0.0),
    )
    automatic = route_electrical_heaters(
        component,
        schematic,
        ElectricalRoutingConfig(**base_kwargs, pad_origin_x_um=None),
    )

    assert automatic.pad_plan.origin_x_um != forced.pad_plan.origin_x_um
    assert not any(
        issue.code == "cross_net_metal_overlap" for issue in automatic.verification.issues
    )
    automatic_failed_terminal_ids = {
        issue.details["terminal_id"]
        for issue in automatic.verification.issues
        if issue.code == "failed_detailed_route"
    }
    # the automatic origin compacts the pad row against the bus escape;
    # before the eight-row-bundles plan's Rule B, that last pad's approach
    # sat inside the escape's realized 400 um band with no legal approach
    # left (the wire clipped the escape undetected, recorded in the
    # ExecPlan, Milestone 2). Rule B leaves enough empty slots between the
    # last individual pad and the bus pad for the band to clear it, so this
    # terminal routes cleanly now.
    assert automatic_failed_terminal_ids == set()
    # pads forced to origin 0 lie far left of every bundle; the old
    # per-bundle nesting construction refused them and the per-wire fallback
    # served only part of them (ExecPlan, Milestone 2). The channel router's
    # joint track assignment (2026-09-29 channel routing plan) routes every
    # wire even at this far-off origin -- every wire just needs a longer
    # detour, not a refusal -- so the compactness check below (average
    # centerline length) is what distinguishes the two origins now, not
    # route count.
    assert not any(issue.code == "cross_net_metal_overlap" for issue in forced.verification.issues)
    assert not any(issue.code == "failed_detailed_route" for issue in forced.verification.issues)
    assert (
        automatic.verification.metrics["pad_channel_height_um"]
        < forced.verification.metrics["pad_channel_height_um"]
    )
    # Every wire routes under both origins now (asserted above), so a raw
    # total would also measure compactness directly; the per-routed-wire
    # average is kept since it is invariant to route count either way.
    automatic_average_length_um = automatic.verification.metrics["centerline_length_um"] / len(
        automatic.detailed_bundle_routes.routes
    )
    forced_average_length_um = forced.verification.metrics["centerline_length_um"] / len(
        forced.detailed_bundle_routes.routes
    )
    assert automatic_average_length_um < forced_average_length_um
    automatic_individual = [
        assignment
        for assignment in automatic.pad_plan.assignments
        if assignment.kind == "individual"
    ]
    assert [assignment.slot.center[0] for assignment in automatic_individual] == sorted(
        assignment.slot.center[0] for assignment in automatic_individual
    )
    for assignment in automatic_individual:
        assert assignment.slot.center[0] == (
            automatic.pad_plan.origin_x_um + assignment.slot.index * automatic.pad_plan.pitch_um
        )

    def exit_distance(result):
        grid = result.obstacle_map.grid
        exits_by_terminal_id = {
            route.terminal.id: grid.origin[0] + (route.exit_cell[0] + 0.5) * grid.grid_size_um
            for route in result.individual_topology.routes
            if route.exit_cell is not None
        }
        return sum(
            abs(assignment.slot.center[0] - exits_by_terminal_id[assignment.terminal.id])
            for assignment in result.pad_plan.assignments
            if assignment.kind == "individual"
        )

    assert exit_distance(automatic) < exit_distance(forced)


def test_auto_pad_channel_height_uses_the_channel_routers_track_count(monkeypatch):
    """The pad plan sizes the channel from the same joint track assignment
    the channel router runs (second slice of the 2026-09-29 channel routing
    plan): the channel height is the router's track count times the track
    pitch plus one wire width -- not the widest bundle's row count (which
    the global count can exceed, cascade_12_folded) and not the global
    route count (which would leave most of the channel empty)."""

    import photonic_router.electrical.river_routing as rr

    track_counts: list[int] = []
    original_assign_tracks = rr.assign_tracks

    def spy(wires, pitch_cells):
        placed, unplaced = original_assign_tracks(wires, pitch_cells)
        track_counts.append(max(placed.values(), default=0))
        return placed, unplaced

    monkeypatch.setattr(rr, "assign_tracks", spy)

    schematic = build_ripup_reroute_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(pad_side="top")

    result = route_electrical_heaters(component, schematic, config)

    assert result.verification is not None
    assert result.verification.success
    assert result.individual_topology is not None
    assert result.detailed_bundle_routes is not None
    assert result.detailed_bundle_routes.success
    assert result.detailed_bundle_routes.bundle_fallback_reasons == {}
    assert track_counts, "expected the channel router to assign tracks"

    from photonic_router.electrical.wire_geometry import wire_reservation_radius_cells

    individual_route_count = len(result.detailed_bundle_routes.routes)
    router_track_count = track_counts[-1]
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    # one track pitch per track, plus the lowest track's reservation radius
    # and one cell of rounding above the layout's top row
    # (``pad_slots._required_pad_channel_offset_um``)
    headroom_um = (
        wire_reservation_radius_cells(result.obstacle_map, config) + 1
    ) * result.obstacle_map.grid.grid_size_um
    channel_height_um = router_track_count * track_pitch_um + headroom_um
    global_channel_height_um = individual_route_count * track_pitch_um + headroom_um

    assert result.verification.metrics["pad_channel_height_um"] == pytest.approx(channel_height_um)
    assert channel_height_um < global_channel_height_um
    assert (
        result.verification.metrics["cross_net_min_spacing_um"]
        >= (result.verification.metrics["required_cross_net_clearance_um"])
    )
    assert result.verification.metrics["centerline_length_um"] < 30_000.0


def test_auto_pad_assignment_is_monotone_in_the_lanes_and_centered_on_each_bundle():
    """Pad-choice plan (2026-10-05): every individual pad follows its wire's
    corridor lane in order (pads strictly increasing with lanes, over all
    bundles), and a multi-row bundle gets its pads on both sides of its lane
    band -- half on each side for an even row count -- instead of one block
    on its exit side.
    """

    import photonic_router.electrical.river_routing as rr

    captured: list = []
    original = rr.assign_tracks

    def spy(wires, pitch_cells):
        captured.append(list(wires))
        return original(wires, pitch_cells)

    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    rr.assign_tracks = spy
    try:
        result = route_electrical_heaters(
            component,
            schematic,
            ElectricalRoutingConfig(
                pad_side="top",
                pad_pitch_um=150.0,
                pad_origin_x_um=None,
                routing_grid_pitch_um=20.0,
                obstacle_clearance_um=10.0,
                terminal_open_radius_um=20.0,
                wire_width_um=20.0,
                individual_route_spacing_um=20.0,
            ),
        )
    finally:
        rr.assign_tracks = original

    assert result.verification is not None
    assert result.verification.success, result.verification.issues
    assert result.detailed_bundle_routes.bundle_fallback_reasons == {}
    wires = captured[-1]
    by_lane = sorted(wires, key=lambda wire: wire.c)
    pads = [wire.p for wire in by_lane]
    assert pads == sorted(pads) and len(set(pads)) == len(pads)

    bundle_by_terminal_id = {
        assignment.terminal.id: assignment.topology_bundle_id
        for assignment in result.pad_plan.assignments
        if assignment.kind == "individual"
    }
    wires_by_bundle: dict = {}
    for wire in wires:
        wires_by_bundle.setdefault(bundle_by_terminal_id[wire.wire_id], []).append(wire)
    multi_row = [group for group in wires_by_bundle.values() if len(group) >= 4]
    assert multi_row
    for group in multi_row:
        inward = sum(1 for wire in group if (wire.p - wire.c) * (group[0].p - group[0].c) < 0)
        assert 0 < inward < len(group), [(wire.wire_id, wire.c, wire.p) for wire in group]


# Also marked e2e (routes a full 16-mode cascade benchmark end to end,
# ~15 s): keeps the fast "-m not e2e" suite under its enforced 60 s budget
# (scripts/test_baseline.sh).
@pytest.mark.integration
@pytest.mark.e2e
def test_every_cascade_wire_channel_routes_with_short_runs(electrical_benchmark_result):
    """Pad-choice plan (2026-10-05): with the pads assigned in lane order by
    the minimum run sum, no wire of ``cascade_16_diagonal`` (seventeen
    escape bundles, up to eight rows) falls back, every horizontal run is at
    most four pad pitches (measured 390 um; the one-sided blocks had 800 um),
    and the channel holds four tracks (measured; the blocks needed ten).
    """

    import photonic_router.electrical.river_routing as rr

    captured: list = []
    original = rr.assign_tracks

    def spy(wires, pitch_cells):
        placed, unplaced = original(wires, pitch_cells)
        captured.append((list(wires), placed, unplaced))
        return placed, unplaced

    rr.assign_tracks = spy
    try:
        result = electrical_benchmark_result("cascade_16_diagonal")
    finally:
        rr.assign_tracks = original

    assert result.verification is not None
    assert result.verification.success, result.verification.issues
    assert result.detailed_bundle_routes.bundle_fallback_reasons == {}
    wires, placed, unplaced = captured[-1]
    assert unplaced == []
    grid_size = result.obstacle_map.grid.grid_size_um
    config = ElectricalRoutingConfig()
    assert max(abs(wire.c - wire.p) for wire in wires) * grid_size <= 4 * config.pad_pitch_um
    assert max(placed.values()) == 4


def test_bus_pad_keeps_escape_clear_of_individual_pads(electrical_benchmark_result):
    """Rule B (eight-row-bundles plan): the empty-slot gap between the last
    individual pad and the bus pad matches ``pad_slots.common_bus_pad_slot_gap``,
    and no individual pad's own bondpad rectangle overlaps a cell of the bus
    escape's realized band (``metal_realization.common_bus_escape_rects``).
    """

    from photonic_router.electrical.metal_realization import common_bus_escape_rects
    from photonic_router.electrical.pad_slots import common_bus_pad_slot_gap
    from photonic_router.electrical.pitch_grid import bbox_to_grid_cells

    result = electrical_benchmark_result("heater_lanes_20")
    config = ElectricalRoutingConfig()
    pad_plan = result.pad_plan
    individual_indices = [
        assignment.slot.index
        for assignment in pad_plan.assignments
        if assignment.kind == "individual"
    ]
    bus_assignment = pad_plan.common_bus_assignment
    assert bus_assignment is not None

    # at least the escape gap: the bus pad also keeps the layout's own edge
    # margin (``_common_bus_pad_slot_index``), which wins once the pad row is
    # compact (pad-choice plan, 2026-10-05)
    assert bus_assignment.slot.index - max(individual_indices) >= common_bus_pad_slot_gap(config)

    escape_rects = common_bus_escape_rects(
        result.common_bus, result.common_bus_escape, result.obstacle_map, config
    )
    escape_cells = set()
    for rect in escape_rects:
        escape_cells |= bbox_to_grid_cells(rect, result.obstacle_map.grid)
    assert escape_cells

    for assignment in pad_plan.assignments:
        if assignment.kind != "individual":
            continue
        pad_cells = bbox_to_grid_cells(assignment.slot.bbox, result.obstacle_map.grid)
        assert not (pad_cells & escape_cells), (
            f"{assignment.terminal.id if assignment.terminal else assignment.net_id}'s pad "
            "overlaps the bus escape's realized band"
        )


# --- assign_pad_slots: pure unit tests on synthetic lane sets (pad-choice
# plan, 2026-10-05). Lanes and slots are bare grid-cell columns; no obstacle
# map, no benchmark.


def test_assign_pad_slots_centers_an_eight_row_bundle_on_its_lane_band():
    """The measured cascade_16_folded bundle: eight lanes three cells apart
    (a 30 um track pitch on a 10 um grid) against a thirteen-cell pad pitch.
    The minimum run sum puts four pads on each side of the lane band and no
    run longer than 400 um (40 cells); the old one-sided block had 810 um.
    """

    from photonic_router.electrical.pad_slots import assign_pad_slots

    lanes = [652 + 3 * i for i in range(8)]
    slots = [500 + 13 * k for k in range(40)]

    indices = assign_pad_slots(lanes, slots)

    assert indices == sorted(indices) and len(set(indices)) == 8
    runs = [slots[k] - lane for lane, k in zip(lanes, indices, strict=True)]
    assert sum(1 for run in runs if run < 0) == 4
    assert sum(1 for run in runs if run > 0) == 4
    assert max(abs(run) for run in runs) <= 40
    # the measured optimum on cascade_16_folded's bundle: 1,600 um of runs
    assert sum(abs(run) for run in runs) == 160


def test_assign_pad_slots_keeps_two_facing_bundles_in_lane_order():
    """A right-exiting bundle (lanes 100..106) and a left-exiting one
    (lanes 200..206) share the slots between them: all pads strictly
    increasing with the lanes, none of the second bundle's pads left of any
    of the first's.
    """

    from photonic_router.electrical.pad_slots import assign_pad_slots

    lanes = [100, 103, 106, 200, 203, 206]
    slots = [0 + 13 * k for k in range(30)]

    indices = assign_pad_slots(lanes, slots)

    assert indices == sorted(indices) and len(set(indices)) == 6
    assert slots[indices[2]] < slots[indices[3]]
    assert all(abs(slots[k] - lane) <= 26 for lane, k in zip(lanes, indices, strict=True))


def test_assign_pad_slots_honours_the_empty_slot_gap_and_refuses_too_few_slots():
    from photonic_router.electrical.pad_slots import assign_pad_slots

    lanes = [10, 11, 12]
    slots = [13 * k for k in range(10)]

    indices = assign_pad_slots(lanes, slots, gap_slots=1)

    assert all(b - a >= 2 for a, b in pairwise(indices))
    with pytest.raises(ValueError):
        assign_pad_slots(lanes, slots[:4], gap_slots=1)


def test_assign_pad_slots_with_origin_finds_the_offset_that_lands_pads_on_lanes():
    """Lanes exactly one pad pitch apart: some origin offset puts every pad
    on its lane (run sum 0); the origin search finds it, and the first
    assigned index is the configured left margin.
    """

    from photonic_router.electrical.pad_slots import assign_pad_slots_with_origin

    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(pad_side="top", pad_pitch_um=130.0, pad_extra_slots_left=2)
    result = route_electrical_heaters(component, schematic, config)
    obstacle_map = result.obstacle_map
    grid_size = obstacle_map.grid.grid_size_um
    pitch_cells = round(130.0 / grid_size)

    lanes = [7 + pitch_cells * i for i in range(5)]
    origin_x_um, indices = assign_pad_slots_with_origin(lanes, obstacle_map, config)

    assert indices == [2, 3, 4, 5, 6]
    # the pad column as the channel router reads it: the slot's access cells'
    # center column (the same chain ``assign_pad_slots_with_origin`` uses)
    from photonic_router.electrical.pad_slots import _make_slot, pad_access_cells
    from photonic_router.electrical.river_routing import pad_column_cell

    for lane, index in zip(lanes, indices, strict=True):
        slot = _make_slot(index, origin_x_um, obstacle_map, config, config.pad_offset_um)
        assert pad_column_cell(pad_access_cells(slot, obstacle_map, config), config)[0] == lane
