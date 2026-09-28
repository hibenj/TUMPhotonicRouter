"""Tests for translation/electrical/individual_topology.py.

Stage 5 (``EscapeTopologyPlanner``): inferring coarse individual escape
corridors, grouped into bundles, before pad assignment.
"""

from __future__ import annotations

from tests.fixtures.sessions import route_electrical_benchmark_result
from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.individual_topology import bundle_route_side
from translation.electrical.types import terminal_exit_dx
from translation.layout_from_schematic import layout_from_schematic


def test_individual_topology_groups_escape_corridors_before_pad_assignment():
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

    assert result.individual_topology is not None
    topology = result.individual_topology
    assert topology.success
    assert len(topology.routes) == 20
    assert len(topology.failed_routes) == 0
    assert topology.shared_cells
    # heater_extra_3 uses its :l terminal here (was :r): Milestone 4's
    # pad-side consistency swap picked the other terminal for its bus
    # branch, since it is not in a bus column group and its original
    # individual terminal exited away from its pad. heater_extra_2 would
    # have qualified the same way, but its original (:r) bus branch is the
    # one heater_extra_1's branch joins onto (a greedy-tree join, not a bus
    # column group); swapping it would have stranded heater_extra_1 with no
    # path to the stripe, so the eight-row-bundles plan's Rule C skips it
    # and it keeps its original :r terminal on the bus.
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
        "heater_extra_3:l",
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
        assignment for assignment in result.pad_plan.assignments if assignment.kind == "individual"
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


def test_bundle_route_side_follows_the_ports():
    """Rule G (2026-09-28, eight-row-bundles plan): a bundle whose member
    terminals all exit the same physical side (``side_key`` "l"/"r") takes
    that side, regardless of where the coarse escape-topology route
    happened to run. Only a bundle with mixed or unknown side keys falls
    back to the exit-interval comparison.

    Checked on ``cascade_16_folded``, which has the lone heater
    ``heater_extra_0:r`` (bundle 2) whose escape-topology route ran
    leftward although its port -- and its pad -- are on the right; before
    Rule G, ``bundle_route_side`` returned "left" for it.
    """

    result = route_electrical_benchmark_result("cascade_16_folded")
    assert result.individual_topology is not None
    topology = result.individual_topology
    obstacle_map = result.obstacle_map

    side_by_bundle_id: dict[int, str] = {}
    for bundle in topology.bundles:
        side_keys = {terminal.side_key for terminal in bundle.ordered_terminals}
        route_side = bundle_route_side(bundle, obstacle_map)
        side_by_bundle_id[bundle.bundle_id] = route_side
        if len(side_keys) != 1:
            continue
        (side_key,) = side_keys
        expected_side = "left" if terminal_exit_dx(side_key) < 0 else "right"
        assert route_side == expected_side, (bundle.bundle_id, side_keys, route_side)

    lone_heater_bundle = next(
        bundle
        for bundle in topology.bundles
        if [terminal.id for terminal in bundle.ordered_terminals] == ["heater_extra_0:r"]
    )
    assert side_by_bundle_id[lone_heater_bundle.bundle_id] == "right"
