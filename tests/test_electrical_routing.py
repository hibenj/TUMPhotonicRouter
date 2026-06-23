from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from gdsfactory.component import Component

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from benchmarks.mmi_heater import build_schematic as build_single_heater_schematic
from benchmarks.mmi_heater_8x4 import build_schematic as build_multi_heater_schematic
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.bundle_detail_router import (
    _offset_path_by_local_normals,
    _realize_ordered_bundle_lanes,
)
from translation.electrical.obstacle_extraction import build_electrical_obstacle_map
from translation.electrical.pad_slots import pad_access_bbox
from translation.electrical.pitch_grid import disk_cells
from translation.electrical.terminal_contacts import select_terminal_contact, terminal_access_path
from translation.electrical.terminal_contacts import terminal_contact_seed_points
from translation.electrical.terminal_extraction import extract_heater_terminal_pairs
from translation.electrical.types import (
    BusStripe,
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalPortRef,
    ElectricalTerminal,
    PadAssignment,
    PadPlan,
    PadSlot,
    TerminalBusRoute,
)
from translation.electrical.verification import verify_electrical_routing
from translation.layout_from_schematic import layout_from_schematic


def _dbu_bbox(bbox_um):
    return tuple(round(value * 1000) for value in bbox_um)


def _polygon_bboxes_by_layer(component: Component, layer: tuple[int, int]):
    return {
        (
            polygon.bbox().left,
            polygon.bbox().bottom,
            polygon.bbox().right,
            polygon.bbox().top,
        )
        for polygon in component.get_polygons(by="tuple").get(layer, [])
    }


def _verification_obstacle_map(
    *,
    raw_obstacle_bboxes: tuple[tuple[float, float, float, float], ...] = (),
) -> ElectricalObstacleMap:
    grid = SimpleNamespace(
        width=40,
        height=40,
        grid_size_um=10.0,
        origin=(0.0, 0.0),
    )
    bus = BusStripe(
        side="bottom",
        bbox=(0.0, 0.0, 40.0, 10.0),
        cells=frozenset({(0, 0), (1, 0), (2, 0), (3, 0)}),
    )
    return ElectricalObstacleMap(
        grid=grid,
        raw_blocked_cells=frozenset(),
        blocked_cells=frozenset(),
        terminal_open_cells={},
        bus=bus,
        die_bbox=(0.0, 0.0, 400.0, 400.0),
        layout_bbox=(0.0, 0.0, 200.0, 200.0),
        raw_obstacle_bboxes=raw_obstacle_bboxes,
    )


def _terminal(terminal_id: str, center: tuple[float, float]) -> ElectricalTerminal:
    port = ElectricalPortRef(
        name="e1",
        center=center,
        orientation=None,
        width=4.0,
        layer=(49, 0),
    )
    return ElectricalTerminal(
        id=terminal_id,
        heater_id=terminal_id.split(":", 1)[0],
        side_key=terminal_id.split(":", 1)[1],
        center=center,
        bbox=(
            center[0] - 2.0,
            center[1] - 2.0,
            center[0] + 2.0,
            center[1] + 2.0,
        ),
        ports=(port,),
        layer=(49, 0),
    )


def test_verifier_models_terminal_landing_contact():
    obstacle_map = _verification_obstacle_map()
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=4.0,
        bus_width_um=4.0,
    )
    terminal = _terminal("heater_0:l", (5.0, 120.0))
    common_bus = CommonBusRoutingResult(
        bus_side="bottom",
        bus=obstacle_map.bus,
        selected_terminals={"heater_0": terminal},
        unselected_terminals={},
        routes=(
            TerminalBusRoute(
                heater_id="heater_0",
                terminal=terminal,
                path=((20, 20), (20, 19), (20, 18)),
                cost=2,
            ),
        ),
        tree_cells=frozenset(obstacle_map.bus.cells),
    )

    verification = verify_electrical_routing(
        obstacle_map,
        common_bus,
        common_bus_escape=None,
        detailed_bundle_routes=None,
        pad_plan=None,
        config=config,
    )

    issue_codes = {issue.code for issue in verification.issues}
    assert "missing_terminal_contact" not in issue_codes


def test_terminal_contact_selects_physical_port_not_logical_center():
    terminal = ElectricalTerminal(
        id="heater_0:l",
        heater_id="heater_0",
        side_key="l",
        center=(50.0, 50.0),
        bbox=(38.0, 38.0, 62.0, 62.0),
        ports=(
            ElectricalPortRef("l_e1", (40.0, 50.0), 180.0, 4.0, (49, 0)),
            ElectricalPortRef("l_e2", (50.0, 60.0), 90.0, 4.0, (49, 0)),
            ElectricalPortRef("l_e3", (60.0, 50.0), 0.0, 4.0, (49, 0)),
            ElectricalPortRef("l_e4", (50.0, 40.0), 270.0, 4.0, (49, 0)),
        ),
        layer=(49, 0),
    )

    contact_center, contact_bbox = select_terminal_contact(
        terminal,
        route_start_um=(0.0, 50.0),
        fallback_width_um=20.0,
    )

    assert contact_center == (40.0, 50.0)
    assert contact_center != terminal.center
    assert contact_bbox == (38.0, 48.0, 42.0, 52.0)


def test_terminal_access_path_trims_snapped_points_inside_terminal():
    terminal = ElectricalTerminal(
        id="heater_0:l",
        heater_id="heater_0",
        side_key="l",
        center=(50.0, 50.0),
        bbox=(38.0, 38.0, 62.0, 62.0),
        ports=(
            ElectricalPortRef("l_e2", (50.0, 60.0), 90.0, 4.0, (49, 0)),
        ),
        layer=(49, 0),
    )

    access = terminal_access_path(
        terminal,
        route_points_um=((50.0, 60.0), (50.0, 65.0), (50.0, 90.0)),
        fallback_width_um=10.0,
    )

    assert access.contact_center == (50.0, 60.0)
    assert access.access_width_um == 4.0
    assert access.adapter_points == ((50.0, 60.0), (50.0, 67.0), (50.0, 90.0))
    assert access.route_tail_points == ((50.0, 90.0),)


def test_verifier_flags_cross_net_metal_overlap():
    obstacle_map = _verification_obstacle_map()
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=10.0,
        bus_width_um=10.0,
    )
    terminal = _terminal("heater_0:r", (20.0, 20.0))
    pad_slot = PadSlot(
        index=0,
        center=(20.0, 300.0),
        bbox=(-20.0, 260.0, 60.0, 340.0),
        side="top",
    )
    pad_assignment = PadAssignment(
        slot=pad_slot,
        net_id="individual:heater_0",
        kind="individual",
        terminal=terminal,
        heater_id="heater_0",
    )
    common_bus = CommonBusRoutingResult(
        bus_side="bottom",
        bus=obstacle_map.bus,
        selected_terminals={},
        unselected_terminals={"heater_0": terminal},
        routes=(),
        tree_cells=frozenset(obstacle_map.bus.cells),
    )
    common_bus_escape = CommonBusEscapeResult(
        pad_assignment=None,
        path=(),
        target_cells=frozenset(),
        success=False,
        reason="not relevant for overlap test",
    )
    detailed_routes = DetailedBundleRoutingResult(
        routes=(
            DetailedBundleRoute(
                bundle_id=0,
                rank=0,
                terminal=terminal,
                pad_assignment=pad_assignment,
                path=((0, 0), (1, 0), (2, 0)),
                target_cells=frozenset({(2, 26)}),
                track_cell=(1, 0),
                lane_cell=(2, 0),
                offset_um=0.0,
                offset_axis="x",
                offset_path=((0.5, 0.5), (2.5, 0.5)),
                success=True,
            ),
        ),
        failed_routes=(),
        committed_cells=frozenset({(0, 0), (1, 0), (2, 0)}),
    )

    verification = verify_electrical_routing(
        obstacle_map,
        common_bus,
        common_bus_escape=common_bus_escape,
        detailed_bundle_routes=detailed_routes,
        pad_plan=PadPlan(
            side="top",
            pitch_um=130.0,
            origin_x_um=0.0,
            slots=(pad_slot,),
            assignments=(pad_assignment,),
            empty_slots=(),
        ),
        config=config,
    )

    issue_codes = {issue.code for issue in verification.issues}
    assert not verification.success
    assert "cross_net_metal_overlap" in issue_codes


def test_verifier_flags_raw_physical_obstacle_overlap_outside_port_contact():
    obstacle_map = _verification_obstacle_map(
        raw_obstacle_bboxes=((30.0, 15.0, 40.0, 25.0),),
    )
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=10.0,
        bus_width_um=10.0,
    )
    terminal = _terminal("heater_0:r", (20.0, 20.0))
    pad_slot = PadSlot(
        index=0,
        center=(50.0, 300.0),
        bbox=(10.0, 260.0, 90.0, 340.0),
        side="top",
    )
    pad_assignment = PadAssignment(
        slot=pad_slot,
        net_id="individual:heater_0",
        kind="individual",
        terminal=terminal,
        heater_id="heater_0",
    )
    common_bus = CommonBusRoutingResult(
        bus_side="bottom",
        bus=obstacle_map.bus,
        selected_terminals={},
        unselected_terminals={"heater_0": terminal},
        routes=(),
        tree_cells=frozenset(obstacle_map.bus.cells),
    )
    detailed_routes = DetailedBundleRoutingResult(
        routes=(
            DetailedBundleRoute(
                bundle_id=0,
                rank=0,
                terminal=terminal,
                pad_assignment=pad_assignment,
                path=((2, 2), (5, 2)),
                target_cells=frozenset({(5, 26)}),
                track_cell=(2, 2),
                lane_cell=(5, 2),
                offset_um=0.0,
                offset_axis="x",
                offset_path=((2.0, 2.0), (5.0, 2.0)),
                success=True,
            ),
        ),
        failed_routes=(),
        committed_cells=frozenset({(2, 2), (5, 2)}),
    )

    verification = verify_electrical_routing(
        obstacle_map,
        common_bus,
        common_bus_escape=CommonBusEscapeResult(
            pad_assignment=None,
            path=(),
            target_cells=frozenset(),
            success=False,
            reason="not relevant for physical overlap test",
        ),
        detailed_bundle_routes=detailed_routes,
        pad_plan=PadPlan(
            side="top",
            pitch_um=130.0,
            origin_x_um=0.0,
            slots=(pad_slot,),
            assignments=(pad_assignment,),
            empty_slots=(),
        ),
        config=config,
    )

    issue_codes = {issue.code for issue in verification.issues}
    assert "metal_overlaps_raw_obstacle" in issue_codes


def test_extracts_two_logical_terminals_from_multi_port_heater():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)

    groups = extract_heater_terminal_pairs(component, schematic, ElectricalRoutingConfig())

    assert len(groups) == 1
    group = groups[0]
    assert group.heater_id == "heater_0"
    assert {group.terminal_a.side_key, group.terminal_b.side_key} == {"l", "r"}
    assert len(group.terminal_a.ports) == 4
    assert len(group.terminal_b.ports) == 4
    assert group.terminal_a.center[0] < group.terminal_b.center[0]


def test_obstacle_map_uses_role_specific_terminal_openings():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig()
    groups = extract_heater_terminal_pairs(component, schematic, config)

    obstacle_map = build_electrical_obstacle_map(component, groups, config)
    terminal = groups[0].terminal_a
    all_port_cells: set[tuple[int, int]] = set()
    for point in terminal_contact_seed_points(terminal):
        all_port_cells.update(
            disk_cells(point, config.terminal_open_radius_um, obstacle_map.grid)
        )
    common_cells = obstacle_map.common_bus_terminal_open_cells[terminal.id]
    individual_cells = obstacle_map.individual_terminal_open_cells[terminal.id]

    assert common_cells
    assert individual_cells
    assert set(common_cells) != all_port_cells
    assert set(individual_cells) != all_port_cells
    assert obstacle_map.terminal_open_cells[terminal.id] == frozenset(
        set(common_cells) | set(individual_cells)
    )


def test_pad_side_derives_opposite_common_bus_side():
    assert ElectricalRoutingConfig(pad_side="top").bus_side == "bottom"
    assert ElectricalRoutingConfig(pad_side="bottom").bus_side == "top"


def test_common_bus_router_selects_exactly_one_terminal_per_heater(tmp_path):
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(
        component,
        schematic,
        config,
        debug_dir=tmp_path,
        debug_prefix="mmi8x4",
    )

    group_ids = {group.heater_id for group in result.terminal_groups}
    assert len(group_ids) == 20
    assert result.common_bus.success
    assert set(result.common_bus.selected_terminals) == group_ids
    assert set(result.common_bus.unselected_terminals) == group_ids
    for group in result.terminal_groups:
        selected = result.common_bus.selected_terminals[group.heater_id]
        unselected = result.common_bus.unselected_terminals[group.heater_id]
        assert selected.id != unselected.id
        assert {selected.id, unselected.id} == {group.terminal_a.id, group.terminal_b.id}
    for route in result.common_bus.routes:
        other_terminal_cells = set().union(
            *(
                cells
                for terminal_id, cells in result.obstacle_map.common_bus_terminal_open_cells.items()
                if terminal_id != route.terminal.id
            )
        )
        assert not set(route.path).intersection(other_terminal_cells)

    svg_path = tmp_path / "electrical" / "mmi8x4_common_bus.svg"
    assert svg_path.exists()
    svg = svg_path.read_text(encoding="utf-8")
    assert svg.startswith("<svg")
    assert "<polyline" in svg
    assert "detailed bundle=" in svg
    assert "topology bundle=" not in svg
    assert "individual route pad=" not in svg


def test_common_bus_terminal_selection_prefers_local_same_row_pair_midpoint():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        common_bus_terminal_selection="local_pair_median_x_biased",
    )

    result = route_electrical_heaters(component, schematic, config)

    groups_by_id = {group.heater_id: group for group in result.terminal_groups}
    expected_pairs = (
        ("heater_0", "heater_post_0"),
        ("heater_1", "heater_post_1"),
        ("heater_2", "heater_post_2"),
        ("heater_3", "heater_post_3"),
        ("heater_output_0", "heater_final_0"),
        ("heater_output_1", "heater_final_1"),
        ("heater_output_2", "heater_final_2"),
        ("heater_output_3", "heater_final_3"),
        ("heater_extra_1", "heater_extra_2"),
    )
    for left_id, right_id in expected_pairs:
        left_group = groups_by_id[left_id]
        right_group = groups_by_id[right_id]
        left_center_x = (left_group.terminal_a.center[0] + left_group.terminal_b.center[0]) / 2.0
        right_center_x = (right_group.terminal_a.center[0] + right_group.terminal_b.center[0]) / 2.0
        midpoint_x = (left_center_x + right_center_x) / 2.0
        for group in (left_group, right_group):
            selected = result.common_bus.selected_terminals[group.heater_id]
            unselected = result.common_bus.unselected_terminals[group.heater_id]
            assert abs(selected.center[0] - midpoint_x) <= abs(unselected.center[0] - midpoint_x)


def test_common_bus_local_trunk_strategy_creates_middle_trunk_for_pairs():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        common_bus_routing_strategy="local_trunk_then_greedy",
        common_bus_terminal_selection="local_pair_median_x_biased",
    )

    result = route_electrical_heaters(component, schematic, config)

    groups_by_id = {group.heater_id: group for group in result.terminal_groups}
    left_group = groups_by_id["heater_0"]
    right_group = groups_by_id["heater_post_0"]
    left_center_x = (left_group.terminal_a.center[0] + left_group.terminal_b.center[0]) / 2.0
    right_center_x = (right_group.terminal_a.center[0] + right_group.terminal_b.center[0]) / 2.0
    midpoint_x = (left_center_x + right_center_x) / 2.0
    grid = result.obstacle_map.grid
    midpoint_grid_x = int((midpoint_x - grid.origin[0]) // grid.grid_size_um)
    route_cells = set()
    for heater_id in ("heater_0", "heater_post_0", "heater_1", "heater_post_1"):
        route = next(route for route in result.common_bus.routes if route.heater_id == heater_id)
        route_cells.update(route.path)

    trunk_cells = {cell for cell in route_cells if cell[0] == midpoint_grid_x}
    assert len(trunk_cells) >= 4
    assert result.common_bus.selected_terminals["heater_0"].side_key == "r"
    assert result.common_bus.selected_terminals["heater_post_0"].side_key == "l"


def test_pad_plan_assigns_slots_without_realizing_geometry():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
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
    for slot in pad_plan.assigned_slots:
        assert slot.center[0] == slot.index * config.pad_pitch_um


def test_pad_access_is_only_on_chip_facing_pad_edge():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    top_config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        pad_access_depth_um=20.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )
    bottom_config = ElectricalRoutingConfig(
        pad_side="bottom",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        pad_access_depth_um=20.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    top_result = route_electrical_heaters(component, schematic, top_config)
    bottom_result = route_electrical_heaters(component, schematic, bottom_config)

    top_slot = top_result.pad_plan.common_bus_assignment.slot
    bottom_slot = bottom_result.pad_plan.common_bus_assignment.slot
    top_half_width = top_config.wire_width_um / 2.0
    bottom_half_width = bottom_config.wire_width_um / 2.0
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
            obstacle_clearance_um=0.0,
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
            obstacle_clearance_um=0.0,
            terminal_open_radius_um=20.0,
        ),
    )

    near_slot = near.pad_plan.common_bus_assignment.slot
    far_slot = far.pad_plan.common_bus_assignment.slot
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
        obstacle_clearance_um=0.0,
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
    assert pad_plan.common_bus_assignment.slot.index == max(slot.index for slot in pad_plan.slots) - 1
    assert pad_plan.empty_slots
    assert 0 in {slot.index for slot in pad_plan.empty_slots}
    assert max(slot.index for slot in pad_plan.slots) in {
        slot.index for slot in pad_plan.empty_slots
    }
    for assignment in assignments:
        assert assignment.slot.center[0] == assignment.slot.index * config.pad_pitch_um


def test_auto_pad_origin_compacts_row_toward_escape_topology():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    base_kwargs = dict(
        pad_side="top",
        pad_pitch_um=150.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
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
            automatic.pad_plan.origin_x_um
            + assignment.slot.index * automatic.pad_plan.pitch_um
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
            obstacle_clearance_um=0.0,
            terminal_open_radius_um=20.0,
            wire_width_um=20.0,
            individual_route_spacing_um=20.0,
        ),
    )

    individual_assignments = [
        assignment
        for assignment in result.pad_plan.assignments
        if assignment.kind == "individual"
    ]
    assignments_by_bundle = {}
    for assignment in individual_assignments:
        assignments_by_bundle.setdefault(assignment.topology_bundle_id, []).append(assignment)

    for bundle in result.individual_topology.bundles:
        bundle_assignments = assignments_by_bundle[bundle.bundle_id]
        slot_indices = [assignment.slot.index for assignment in bundle_assignments]
        assert slot_indices == list(range(slot_indices[0], slot_indices[0] + bundle.required_tracks))
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


def test_individual_topology_groups_escape_corridors_before_pad_assignment():
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

    assert result.individual_topology is not None
    topology = result.individual_topology
    assert topology.success
    assert len(topology.routes) == 20
    assert len(topology.failed_routes) == 0
    assert topology.shared_cells
    assert [terminal.id for terminal in topology.terminal_order] == [
        "heater_3:l",
        "heater_2:l",
        "heater_1:l",
        "heater_0:l",
        "heater_post_0:r",
        "heater_post_1:r",
        "heater_post_2:r",
        "heater_post_3:r",
        "heater_extra_0:l",
        "heater_extra_1:l",
        "heater_extra_2:r",
        "heater_extra_3:r",
        "heater_output_3:l",
        "heater_output_2:l",
        "heater_output_1:l",
        "heater_output_0:l",
        "heater_final_0:r",
        "heater_final_1:r",
        "heater_final_2:r",
        "heater_final_3:r",
    ]
    for route in topology.routes:
        other_terminal_cells = set().union(
            *(
                cells
                for terminal_id, cells in result.obstacle_map.individual_terminal_open_cells.items()
                if terminal_id != route.terminal.id
            )
        )
        assert not set(route.path).intersection(other_terminal_cells)
    assert [bundle.required_tracks for bundle in topology.bundles] == [
        4,
        4,
        1,
        1,
        1,
        1,
        4,
        4,
    ]
    assert all(bundle.order_axis == "y" for bundle in topology.bundles)

    first_bundle = topology.bundles[0]
    assert first_bundle.required_width_um == 140.0
    assert [terminal.heater_id for terminal in first_bundle.ordered_terminals] == [
        "heater_3",
        "heater_2",
        "heater_1",
        "heater_0",
    ]
    right_bundle = topology.bundles[1]
    assert [terminal.heater_id for terminal in right_bundle.ordered_terminals] == [
        "heater_post_0",
        "heater_post_1",
        "heater_post_2",
        "heater_post_3",
    ]

    individual_assignments = [
        assignment
        for assignment in result.pad_plan.assignments
        if assignment.kind == "individual"
    ]
    first_bundle_assignments = [
        assignment
        for assignment in individual_assignments
        if assignment.topology_bundle_id == first_bundle.bundle_id
    ]
    assert [assignment.topology_rank for assignment in first_bundle_assignments] == [0, 1, 2, 3]
    assert [assignment.terminal.id for assignment in first_bundle_assignments] == [
        terminal.id for terminal in first_bundle.ordered_terminals
    ]
    assert [assignment.slot.index for assignment in first_bundle_assignments] == [0, 1, 2, 3]
    right_bundle_assignments = [
        assignment
        for assignment in individual_assignments
        if assignment.topology_bundle_id == right_bundle.bundle_id
    ]
    assert [assignment.topology_rank for assignment in right_bundle_assignments] == [0, 1, 2, 3]
    assert [assignment.terminal.id for assignment in right_bundle_assignments] == [
        terminal.id for terminal in right_bundle.ordered_terminals
    ]
    assert [assignment.slot.index for assignment in right_bundle_assignments] == [4, 5, 6, 7]


def test_detailed_bundle_router_assigns_spaced_offsets_from_topology():
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
    assert detailed.success
    assert len(detailed.routes) == 20
    assert len(detailed.failed_routes) == 0
    assert detailed.track_pitch_cells == 2

    first_bundle_routes = [
        route for route in detailed.routes if route.bundle_id == result.individual_topology.bundles[0].bundle_id
    ]
    assert [route.rank for route in first_bundle_routes] == [0, 1, 2, 3]
    assert [route.offset_um for route in first_bundle_routes] == [-120.0, -80.0, -40.0, 0.0]
    assert all(route.offset_axis == "x" for route in first_bundle_routes)
    assert [route.pad_assignment.slot.index for route in first_bundle_routes] == [0, 1, 2, 3]
    assert [route.terminal.id for route in first_bundle_routes] == [
        terminal.id for terminal in result.individual_topology.bundles[0].ordered_terminals
    ]
    right_bundle_routes = [
        route for route in detailed.routes if route.bundle_id == result.individual_topology.bundles[1].bundle_id
    ]
    assert [route.terminal.id for route in right_bundle_routes] == [
        terminal.id for terminal in result.individual_topology.bundles[1].ordered_terminals
    ]
    assert [route.offset_um for route in right_bundle_routes] == [0.0, 40.0, 80.0, 120.0]
    assert [route.pad_assignment.slot.index for route in right_bundle_routes] == [4, 5, 6, 7]

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
        assert route.offset_path
        assert route.source_stub_path
        assert route.bundle_track_path
        assert route.pad_stub_path
        assert route.offset_path[0] == route.source_stub_path[0]
        assert route.source_stub_path[-1] == route.bundle_track_path[0]
        assert route.bundle_track_path[-1] == route.pad_stub_path[0]
        assert route.offset_path[-1] == route.pad_stub_path[-1]
        assert route.bundle_track_path[-1][1] == route.pad_stub_path[1][1]
        assert max(point[1] for point in route.bundle_track_path) <= route.pad_stub_path[1][1]
        assert not set(route.path).intersection(other_terminal_cells)
        offset_cells = {
            (round(x - 0.5), round(y - 0.5))
            for x, y in route.offset_path
        }
        hard_blocked = set(result.obstacle_map.blocked_cells)
        hard_blocked.update(result.common_bus.tree_cells)
        hard_blocked.update(result.common_bus_escape.path)
        assert not set(route.path).intersection(hard_blocked)
        assert not offset_cells.intersection(hard_blocked)
        assert not offset_cells.intersection(other_terminal_cells)

    left_outer = first_bundle_routes[0]
    assert left_outer.source_stub_path[0][0] > left_outer.source_stub_path[-1][0]
    right_outer = right_bundle_routes[-1]
    assert right_outer.source_stub_path[0][0] < right_outer.source_stub_path[-1][0]
    assert [route.pad_stub_path[1][1] for route in right_bundle_routes] == sorted(
        route.pad_stub_path[1][1] for route in right_bundle_routes
    )
    assert right_outer.pad_stub_path[1][1] == max(
        route.pad_stub_path[1][1] for route in right_bundle_routes
    )


def test_metal_realization_creates_assigned_pads_but_not_empty_slots():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=None,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    routed_component = result.routed_component
    assert routed_component is not None
    polygon_bboxes = _polygon_bboxes_by_layer(routed_component, config.metal_layer)
    assigned_pad_bboxes = {_dbu_bbox(assignment.slot.bbox) for assignment in result.pad_plan.assignments}
    empty_slot_bboxes = {_dbu_bbox(slot.bbox) for slot in result.pad_plan.empty_slots}

    assert assigned_pad_bboxes
    assert assigned_pad_bboxes.issubset(polygon_bboxes)
    assert not empty_slot_bboxes.intersection(polygon_bboxes)


def test_metal_realization_adds_wire_polygons_for_bus_and_individual_routes():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=None,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.routed_component is not None
    polygons = result.routed_component.get_polygons(by="tuple").get(config.metal_layer, [])
    assigned_pad_bboxes = {_dbu_bbox(assignment.slot.bbox) for assignment in result.pad_plan.assignments}
    wire_width_dbu = round(config.wire_width_um * 1000)
    bus_width_dbu = round(config.bus_width_um * 1000)
    wire_like_bboxes = []
    for polygon in polygons:
        bbox = polygon.bbox()
        bbox_tuple = (bbox.left, bbox.bottom, bbox.right, bbox.top)
        if bbox_tuple in assigned_pad_bboxes:
            continue
        if bbox.width() == wire_width_dbu or bbox.height() == wire_width_dbu:
            wire_like_bboxes.append(bbox_tuple)
        elif bbox.width() == bus_width_dbu or bbox.height() == bus_width_dbu:
            wire_like_bboxes.append(bbox_tuple)

    assert wire_like_bboxes
    assert len(wire_like_bboxes) >= len(result.detailed_bundle_routes.routes)


def test_show_realized_electrical_metal_in_klayout():
    if os.environ.get("SHOW_ELECTRICAL_KLAYOUT") != "1":
        pytest.skip("set SHOW_ELECTRICAL_KLAYOUT=1 to open KLayout")

    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=None,
        pad_offset_um=500.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.routed_component is not None
    assert result.detailed_bundle_routes.success
    result.routed_component.show()
    gds_path = PROJECT_ROOT / "build" / "electrical" / "realized_electrical_metal.gds"
    gds_path.parent.mkdir(parents=True, exist_ok=True)
    result.routed_component.write_gds(gds_path)
    subprocess.Popen(
        ["klayout", str(gds_path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def test_local_normal_offset_rotates_through_bundle_bends():
    path = ((10, 10), (10, 11), (10, 12), (11, 12), (12, 12))

    left_offset = _offset_path_by_local_normals(
        path,
        offset_um=20.0,
        side="left",
        grid_size_um=10.0,
    )
    right_offset = _offset_path_by_local_normals(
        path,
        offset_um=20.0,
        side="right",
        grid_size_um=10.0,
    )

    assert left_offset == (
        (8.5, 10.5),
        (8.5, 12.5),
        (8.5, 14.5),
        (10.5, 14.5),
        (12.5, 14.5),
    )
    assert right_offset == (
        (12.5, 10.5),
        (12.5, 12.5),
        (12.5, 10.5),
        (10.5, 10.5),
        (12.5, 10.5),
    )


def test_ordered_bundle_lanes_preserve_vertical_lane_order():
    lanes = _realize_ordered_bundle_lanes(
        ((10, 10), (10, 12)),
        lane_count=4,
        track_pitch_um=20.0,
        route_side="right",
        grid_size_um=10.0,
    )

    assert [lane[0][0] for lane in lanes] == [10.5, 12.5, 14.5, 16.5]
    assert [lane[-1][0] for lane in lanes] == [10.5, 12.5, 14.5, 16.5]
    assert all(lane[0][1] == 10.5 and lane[-1][1] == 12.5 for lane in lanes)


def test_ordered_bundle_lanes_preserve_horizontal_lane_order():
    lanes = _realize_ordered_bundle_lanes(
        ((10, 10), (12, 10)),
        lane_count=4,
        track_pitch_um=20.0,
        route_side="left",
        grid_size_um=10.0,
    )

    assert [lane[0][1] for lane in lanes] == [16.5, 14.5, 12.5, 10.5]
    assert [lane[-1][1] for lane in lanes] == [16.5, 14.5, 12.5, 10.5]
    assert all(lane[0][0] == 10.5 and lane[-1][0] == 12.5 for lane in lanes)


def test_ordered_bundle_lanes_flip_local_side_at_left_turn():
    lanes = _realize_ordered_bundle_lanes(
        ((10, 10), (10, 20), (0, 20)),
        lane_count=4,
        track_pitch_um=20.0,
        route_side="right",
        grid_size_um=10.0,
    )

    assert [lane[0][0] for lane in lanes] == [10.5, 12.5, 14.5, 16.5]
    assert [lane[-1][1] for lane in lanes] == [20.5, 22.5, 24.5, 26.5]
    assert lanes[3] == ((16.5, 10.5), (16.5, 26.5), (0.5, 26.5))


def test_ordered_bundle_lanes_flip_local_side_at_right_turn():
    lanes = _realize_ordered_bundle_lanes(
        ((10, 10), (10, 20), (20, 20)),
        lane_count=4,
        track_pitch_um=20.0,
        route_side="right",
        grid_size_um=10.0,
    )

    assert [lane[0][0] for lane in lanes] == [10.5, 12.5, 14.5, 16.5]
    assert [lane[-1][1] for lane in lanes] == [20.5, 18.5, 16.5, 14.5]
    assert lanes[3] == ((16.5, 10.5), (16.5, 14.5), (20.5, 14.5))


def test_ordered_bundle_lanes_preserve_lane_identity_through_multiple_bends():
    lanes = _realize_ordered_bundle_lanes(
        ((10, 10), (10, 20), (20, 20), (20, 30), (30, 30)),
        lane_count=3,
        track_pitch_um=20.0,
        route_side="right",
        grid_size_um=10.0,
    )

    assert len(lanes) == 3
    assert [lane[0][0] for lane in lanes] == [10.5, 12.5, 14.5]
    assert [lane[-1][1] for lane in lanes] == [30.5, 28.5, 26.5]
    for lane in lanes:
        for start, end in zip(lane, lane[1:]):
            assert start[0] == end[0] or start[1] == end[1]


def test_common_bus_escape_reaches_assigned_common_bus_pad_slot():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.pad_plan is not None
    assert result.common_bus_escape is not None
    escape = result.common_bus_escape
    assert escape.success, escape.reason
    assert escape.pad_assignment == result.pad_plan.common_bus_assignment
    assert escape.path[0] in result.common_bus.tree_cells
    assert escape.path[-1] in escape.target_cells
    assert escape.target_cells
    assert escape.cost == len(escape.path) - 1


def test_common_bus_escape_uses_opposite_bus_for_bottom_pad_side():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="bottom",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.common_bus.bus_side == "top"
    assert result.pad_plan is not None
    assert result.pad_plan.side == "bottom"
    assert result.common_bus_escape is not None
    assert result.common_bus_escape.success, result.common_bus_escape.reason
    assert result.common_bus_escape.path[0] in result.common_bus.tree_cells
    assert result.common_bus_escape.path[-1] in result.common_bus_escape.target_cells
