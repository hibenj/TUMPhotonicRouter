from benchmarks.multiportmmi_8x8 import build_schematic
import pytest
from routing_flow import RoutingFlowStats, run_routing_flow
from translation.layout_from_schematic import layout_from_schematic


def test_multiportmmi_8x8_builds_from_lidar_netlist():
    schematic = build_schematic()

    assert len(schematic.netlist.instances) == 82
    assert len(schematic.placements) == 82
    assert len(schematic.netlist.routes) == 111

    assert schematic.netlist.instances["gc1"].component == "grating_coupler_elliptical_lumerical"
    assert schematic.netlist.instances["mmi0_multiport_0_0"].component == "mmi"
    assert schematic.netlist.instances["mmi0_multiport_0_0"].settings["inputs"] == 6
    assert schematic.netlist.instances["mmi0_multiport_0_0"].settings["outputs"] == 6

    placement = schematic.placements["mmi0_multiport_0_0"]
    assert placement.x == 1263.8
    assert placement.y == 685.0
    assert placement.port == "sw"

    assert schematic.netlist.routes["n_31"].links == {
        "mmi0_multiport_0_0,o12": "mmi0_ps_array_1_heater_4,o1"
    }


def test_multiportmmi_8x8_unrouted_layout_instantiates():
    schematic = build_schematic()
    layout = layout_from_schematic(schematic)

    assert len(layout.insts) == 82
    bbox = layout.dbbox()
    assert bbox.right > 4300
    assert bbox.top > 1100


@pytest.mark.parametrize(
    (
        "enable_crossings",
        "crossing_mode",
        "expected_route_attempts",
        "expected_route_failures",
        "expected_repair_count",
    ),
    [
        (False, "window", 31, 0, 0),
        # attempts=42 (not 31), failures=8 (not 0), repairs=1 (not 0): fixing
        # try_route_with_collision_crossings_using_primitives's
        # "collision_crossing_route_without_event_is_not_accepted" bug (a route
        # with zero crossing events against its requested partner was being
        # vacuously accepted as a successful collision-crossing result) means
        # some of these first 31 nets no longer get a free pass on their first,
        # non-crossing collision-crossing attempt: that attempt is now correctly
        # counted as a failed attempt (route_failures), and the net falls
        # through to a second, plain-A* attempt (route_attempts) that still
        # routes it cleanly -- exactly the "a failed local collision-crossing
        # attempt must not make the whole net unroutable" fallback this
        # function's own caller already documents and relies on. One net's
        # plain-A* fallback also needed one rip-up/repair round to clear a
        # dynamic blocker (repair_count), which the routing flow already
        # supports and which still ends in a fully clean result (still
        # success=True, error_count=0, 31/31 routed) -- only these internal
        # attempt/failure/repair counters legitimately changed, and their
        # prior values of 0 were themselves a symptom of the bug (every
        # collision-crossing attempt vacuously "succeeded" on the first try,
        # so no attempt ever needed a fallback), not evidence of cleaner
        # routing. See
        # .agent/execplans/2026-08-19-fix-collision-crossing-zero-event-acceptance.md.
        (True, "lidar-pure", 42, 8, 1),
    ],
)
def test_multiportmmi_8x8_routes_cleanly_through_first_mmi_fanin_boundary(
    enable_crossings,
    crossing_mode,
    expected_route_attempts,
    expected_route_failures,
    expected_repair_count,
):
    stats = RoutingFlowStats()

    routed = run_routing_flow(
        "multiportmmi_8x8",
        debug_svgs=False,
        show_unrouted=False,
        show_routed=False,
        show_static_obstacles_svg=False,
        debug_timing=False,
        verbose_routes=False,
        enable_path_length_matching=False,
        path_length_match_outputs=False,
        enable_crossings=enable_crossings,
        crossing_mode=crossing_mode,
        debug_stop_after_route_index=31,
        stats=stats,
    )

    photonic_verification = dict(routed.info["photonic_verification"])
    assert photonic_verification["success"] is True
    assert photonic_verification["error_count"] == 0
    assert photonic_verification["debug_stop_after_route_index"] == 31
    assert photonic_verification["routed_record_count"] == 31
    assert stats.route_attempts == expected_route_attempts
    assert stats.route_failures == expected_route_failures
    assert stats.repair_count == expected_repair_count
