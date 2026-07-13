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
    ("enable_crossings", "crossing_mode"),
    [
        (False, "window"),
        (True, "lidar-pure"),
    ],
)
def test_multiportmmi_8x8_routes_cleanly_through_first_mmi_fanin_boundary(
    enable_crossings,
    crossing_mode,
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
    assert stats.route_attempts == 31
    assert stats.route_failures == 0
    assert stats.repair_count == 0
