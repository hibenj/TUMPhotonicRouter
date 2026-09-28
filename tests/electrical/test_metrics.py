"""Tests for translation/electrical/metrics.py.

The pad-wire quality numbers (length, Manhattan distance, detour, bend
count) reported into ``ElectricalVerificationResult.metrics``.
"""

from __future__ import annotations

import pytest

from tests.electrical.support import _terminal, _verification_obstacle_map
from tests.fixtures.synthetic_layouts import (
    single_heater_schematic as build_single_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.metrics import _bend_count, _polyline_length
from translation.electrical.net_geometry import (
    detailed_route_centerline_points as _detailed_route_centerline_points,
)
from translation.electrical.types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    PadAssignment,
    PadPlan,
    PadSlot,
)
from translation.electrical.verification import verify_electrical_routing
from translation.layout_from_schematic import layout_from_schematic


def test_pad_wire_metrics_report_consistent_length_manhattan_and_bends():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig()

    result = route_electrical_heaters(component, schematic, config)

    assert result.verification is not None
    assert result.detailed_bundle_routes is not None
    pad_wires = result.verification.metrics["pad_wires"]
    assert pad_wires

    routes_by_terminal = {
        route.terminal.id: route for route in result.detailed_bundle_routes.routes
    }
    for entry in pad_wires:
        route = routes_by_terminal[entry["terminal_id"]]
        assert entry["heater_id"] == route.terminal.heater_id
        centerline = _detailed_route_centerline_points(route, result.obstacle_map)
        assert entry["length_um"] == pytest.approx(_polyline_length(centerline))
        assert entry["bend_count"] == _bend_count(centerline)
        assert entry["length_um"] >= entry["manhattan_um"] - 1e-9
        assert entry["detour_um"] == pytest.approx(entry["length_um"] - entry["manhattan_um"])
        assert entry["detour_um"] >= 0.0

    metrics = result.verification.metrics
    assert metrics["pad_wire_detour_total_um"] == pytest.approx(
        sum(entry["detour_um"] for entry in pad_wires)
    )
    assert metrics["pad_wire_max_detour_um"] == pytest.approx(
        max(entry["detour_um"] for entry in pad_wires)
    )
    assert metrics["pad_wire_max_bend_count"] == max(entry["bend_count"] for entry in pad_wires)


def test_pad_wire_metrics_report_detour_for_a_deliberately_jogged_centerline():
    obstacle_map = _verification_obstacle_map()
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=4.0,
        bus_width_um=4.0,
        terminal_contact_width_um=4.0,
    )
    terminal = _terminal("heater_0:l", (45.0, 5.0))
    pad_slot = PadSlot(
        index=0,
        center=(5.0, 25.0),
        bbox=(-5.0, 20.0, 15.0, 30.0),
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
        reason="not relevant for this test",
    )
    # A deliberate up/right/down/right jog: twice the Manhattan distance
    # between its own endpoints, with three bends.
    detailed_routes = DetailedBundleRoutingResult(
        routes=(
            DetailedBundleRoute(
                bundle_id=0,
                rank=0,
                terminal=terminal,
                pad_assignment=pad_assignment,
                path=((0, 0), (2, 0)),
                target_cells=frozenset({(0, 2)}),
                centerline=((0.5, 0.5), (0.5, 2.5), (2.5, 2.5), (2.5, 0.5), (4.5, 0.5)),
                success=True,
            ),
        ),
        failed_routes=(),
        committed_cells=frozenset(),
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

    pad_wires = verification.metrics["pad_wires"]
    assert len(pad_wires) == 1
    entry = pad_wires[0]
    assert entry["length_um"] == pytest.approx(80.0)
    assert entry["manhattan_um"] == pytest.approx(40.0)
    assert entry["detour_um"] == pytest.approx(40.0)
    assert entry["bend_count"] == 3
    assert verification.metrics["pad_wire_detour_total_um"] == pytest.approx(40.0)
    assert verification.metrics["pad_wire_max_detour_um"] == pytest.approx(40.0)
    assert verification.metrics["pad_wire_max_bend_count"] == 3
