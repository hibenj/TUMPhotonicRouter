"""Tests for translation/electrical/verification.py.

Stage 11 (``ElectricalVerifier``): verifying the realized electrical
geometry's contracts -- terminal contacts, cross-net and raw-obstacle
overlap, and common-bus connectivity.
"""

from __future__ import annotations

from tests.electrical.support import _terminal, _verification_obstacle_map
from translation.electrical import ElectricalRoutingConfig
from translation.electrical.types import (
    BusStripe,
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalTerminal,
    PadAssignment,
    PadPlan,
    PadSlot,
    TerminalBusRoute,
)
from translation.electrical.verification import verify_electrical_routing


def test_verifier_models_terminal_landing_contact():
    obstacle_map = _verification_obstacle_map()
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=4.0,
        bus_width_um=4.0,
        terminal_contact_width_um=4.0,
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
    assert verification.metrics["same_net_overlap_pair_count_by_source"] == {
        "bus_route/terminal_adapter": 1,
    }
    assert verification.metrics["same_net_intentional_overlap_pair_count"] == 1
    assert verification.metrics["same_net_redundant_overlap_pair_count"] == 0
    assert verification.metrics["same_net_intentional_overlap_pair_count_by_reason"] == {
        "terminal_access_join": 1,
    }
    assert verification.metrics["metal_area_overcount_um2"] == 16.0
    assert verification.metrics["metal_area_overcount_by_reason_um2"] == {
        "terminal_access_join": 16.0,
    }
    assert verification.metrics["metal_area_overcount_by_source_um2"] == {
        "bus_route/terminal_adapter": 16.0,
    }
    assert verification.metrics["metal_redundant_area_overcount_um2"] == 0


def test_verifier_flags_cross_net_metal_overlap():
    obstacle_map = _verification_obstacle_map()
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=10.0,
        bus_width_um=10.0,
        terminal_contact_width_um=10.0,
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
                centerline=((0.5, 0.5), (2.5, 0.5)),
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


def test_verifier_includes_assigned_pad_rectangles_in_cross_net_overlap():
    obstacle_map = _verification_obstacle_map()
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=10.0,
        bus_width_um=10.0,
    )
    terminal = _terminal("heater_0:r", (20.0, 20.0))
    overlapping_pad_slot = PadSlot(
        index=0,
        center=(50.0, 300.0),
        bbox=(10.0, 260.0, 90.0, 340.0),
        side="top",
    )
    individual_assignment = PadAssignment(
        slot=overlapping_pad_slot,
        net_id="individual:heater_0",
        kind="individual",
        terminal=terminal,
        heater_id="heater_0",
    )
    common_assignment = PadAssignment(
        slot=overlapping_pad_slot,
        net_id="common_bus",
        kind="common_bus",
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
                pad_assignment=individual_assignment,
                path=((2, 2), (5, 2)),
                target_cells=frozenset({(5, 26)}),
                centerline=((2.0, 2.0), (5.0, 2.0)),
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
            pad_assignment=common_assignment,
            path=(),
            target_cells=frozenset(),
            success=False,
            reason="not relevant for pad rectangle overlap test",
        ),
        detailed_bundle_routes=detailed_routes,
        pad_plan=PadPlan(
            side="top",
            pitch_um=130.0,
            origin_x_um=0.0,
            slots=(overlapping_pad_slot,),
            assignments=(individual_assignment, common_assignment),
            empty_slots=(),
        ),
        config=config,
    )

    cross_net_issues = [
        issue for issue in verification.issues if issue.code == "cross_net_metal_overlap"
    ]
    assert cross_net_issues
    assert any(issue.details["other_net_id"] == "individual:heater_0" for issue in cross_net_issues)


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
                centerline=((2.0, 2.0), (5.0, 2.0)),
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


def test_common_bus_verification_reports_a_branch_that_reaches_nothing():
    """A branch ending on a cell of a discarded branch is metal that reaches
    nothing; the verifier must say so, transitively accepting branches that
    reach the stripe through other branches."""

    from translation.electrical.verification import _verify_common_bus_connectivity

    def terminal(identifier: str, heater_id: str, x: float) -> ElectricalTerminal:
        return ElectricalTerminal(
            id=identifier,
            heater_id=heater_id,
            side_key=identifier.rsplit(":", 1)[-1],
            center=(x, 100.0),
            bbox=(x - 5.0, 95.0, x + 5.0, 105.0),
            ports=(),
        )

    stripe = BusStripe(
        side="bottom", bbox=(0.0, 0.0, 200.0, 10.0), cells=frozenset({(x, 0) for x in range(20)})
    )
    first = terminal("h0:l", "h0", 40.0)
    second = terminal("h1:l", "h1", 80.0)
    third = terminal("h2:l", "h2", 120.0)
    routes = (
        TerminalBusRoute(
            heater_id="h0",
            terminal=first,
            path=((4, 5), (4, 4), (4, 3), (4, 2), (4, 1), (4, 0)),
            cost=5,
        ),
        TerminalBusRoute(
            heater_id="h1", terminal=second, path=((8, 5), (7, 5), (6, 5), (5, 5), (4, 5)), cost=4
        ),
        TerminalBusRoute(heater_id="h2", terminal=third, path=((12, 5), (12, 4), (12, 3)), cost=2),
    )
    common_bus = CommonBusRoutingResult(
        bus_side="bottom",
        bus=stripe,
        selected_terminals={"h0": first, "h1": second, "h2": third},
        unselected_terminals={},
        routes=routes,
        tree_cells=frozenset(stripe.cells | {cell for route in routes for cell in route.path}),
    )

    issues: list = []
    _verify_common_bus_connectivity(issues, common_bus)

    assert [issue.code for issue in issues] == ["common_bus_route_disconnected"]
    assert issues[0].details["terminal_id"] == "h2:l"
    assert issues[0].details["route_end_cell"] == (12, 3)
