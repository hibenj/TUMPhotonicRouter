"""Tests for translation/electrical/pad_slots.py.

Stage 6 (``PadPlanner``): assigning used electrical nets to legal pad-pitch
slots, both forced and automatic pad-row placement.
"""

from __future__ import annotations

import pytest

from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from tests.fixtures.synthetic_layouts import (
    ripup_reroute_heater_schematic as build_ripup_reroute_schematic,
)
from tests.fixtures.synthetic_layouts import (
    single_heater_schematic as build_single_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.pad_slots import pad_access_bbox
from translation.layout_from_schematic import layout_from_schematic


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
    base_kwargs = dict(
        pad_side="top",
        pad_pitch_um=150.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=10.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

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
    # the automatic origin compacts the pad row against the bus escape, and
    # with the escape blocked at its realized 400 um width that last pad has
    # no legal approach; on HEAD the wire clipped the escape undetected
    # (recorded in the ExecPlan, Milestone 2).
    assert automatic_failed_terminal_ids == {"heater_final_3:r"}
    # pads forced to origin 0 lie far left of every bundle; the river
    # construction refuses them and the per-wire fallback serves only part
    # of them (ExecPlan, Milestone 2), but at a positive clearance no wire
    # is allowed to touch another net's metal, so there is no overlap issue
    # at all.
    assert not any(issue.code == "cross_net_metal_overlap" for issue in forced.verification.issues)
    assert any(issue.code == "failed_detailed_route" for issue in forced.verification.issues)
    assert (
        automatic.verification.metrics["pad_channel_height_um"]
        < forced.verification.metrics["pad_channel_height_um"]
    )
    assert (
        automatic.verification.metrics["centerline_length_um"]
        < forced.verification.metrics["centerline_length_um"]
    )
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


def test_auto_pad_channel_height_uses_widest_topology_bundle():
    schematic = build_ripup_reroute_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(pad_side="top")

    result = route_electrical_heaters(component, schematic, config)

    assert result.verification is not None
    assert result.verification.success
    assert result.individual_topology is not None
    assert result.detailed_bundle_routes is not None
    assert result.detailed_bundle_routes.success

    individual_route_count = len(result.detailed_bundle_routes.routes)
    widest_bundle_tracks = max(
        bundle.required_tracks for bundle in result.individual_topology.bundles
    )
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    compact_channel_height_um = widest_bundle_tracks * track_pitch_um + config.wire_width_um
    global_channel_height_um = individual_route_count * track_pitch_um + config.wire_width_um

    assert result.verification.metrics["pad_channel_height_um"] == pytest.approx(
        compact_channel_height_um
    )
    assert compact_channel_height_um < global_channel_height_um
    assert (
        result.verification.metrics["cross_net_min_spacing_um"]
        >= (result.verification.metrics["required_cross_net_clearance_um"])
    )
    assert result.verification.metrics["centerline_length_um"] < 30_000.0


def test_auto_pad_assignment_places_topology_bundles_as_intervals_with_gaps():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
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

    individual_assignments = [
        assignment for assignment in result.pad_plan.assignments if assignment.kind == "individual"
    ]
    assignments_by_bundle = {}
    for assignment in individual_assignments:
        assignments_by_bundle.setdefault(assignment.topology_bundle_id, []).append(assignment)

    for bundle in result.individual_topology.bundles:
        bundle_assignments = assignments_by_bundle[bundle.bundle_id]
        slot_indices = [assignment.slot.index for assignment in bundle_assignments]
        assert slot_indices == list(
            range(slot_indices[0], slot_indices[0] + bundle.required_tracks)
        )
        assert [assignment.topology_rank for assignment in bundle_assignments] == list(
            range(bundle.required_tracks)
        )

    bundle_starts = [
        assignments_by_bundle[bundle.bundle_id][0].slot.index
        for bundle in result.individual_topology.bundles
    ]
    bundle_ends = [
        assignments_by_bundle[bundle.bundle_id][-1].slot.index
        for bundle in result.individual_topology.bundles
    ]
    assert bundle_starts == sorted(bundle_starts)
    assert any(
        next_start > previous_end + 1
        for previous_end, next_start in zip(bundle_ends, bundle_starts[1:])
    )
