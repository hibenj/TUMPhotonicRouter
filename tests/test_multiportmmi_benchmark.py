from benchmarks.multiportmmi_8x8 import build_schematic
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
