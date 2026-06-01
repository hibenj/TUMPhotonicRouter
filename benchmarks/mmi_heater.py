"""Minimal MMI-heater-MMI benchmark with logical nets only (no routing)."""

from gdsfactory.generic_tech import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

NODE_TYPES = {
    "gc_0": "input",
    "gc_1": "input",
    "mmi_0": "gate",
    "heater_0": "gate",
    "mmi_1": "gate",
    "gc_2": "output",
    "gc_3": "output",
}

INTERNAL_DELAYS_UM = {
    "gc_0": 0.0,
    "gc_1": 0.0,
    "mmi_0": 0.0,
    "heater_0": 0.0,
    "mmi_1": 0.0,
    "gc_2": 0.0,
    "gc_3": 0.0,
}


def build_schematic() -> Schematic:
    """Build a minimal placement + netlist benchmark without any routing.

    Topology:
        input GCs -> mmi_0 -> heater_0 -> mmi_1 -> output GCs

    Note:
        This benchmark intentionally only defines component placement and
        logical connections. It does not perform waveguide routing.
    """
    pdk = get_generic_pdk()
    pdk.activate()

    schematic = Schematic()

    gc_instance = Instance(component="grating_coupler_te")
    mmi_instance = Instance(component="mmi2x2")
    heater_instance = Instance(component="straight_heater_metal")

    # Left-to-right deterministic placement with generous spacing.
    schematic.add_instance("gc_0", gc_instance, Placement(x=0, y=40, mirror=True))
    schematic.add_instance("gc_1", gc_instance, Placement(x=0, y=-40, mirror=True))
    schematic.add_instance("mmi_0", mmi_instance, Placement(x=120, y=0, rotation=0))
    schematic.add_instance("heater_0", heater_instance, Placement(x=240, y=40, rotation=0))
    schematic.add_instance("mmi_1", mmi_instance, Placement(x=620, y=0, rotation=0))
    schematic.add_instance("gc_2", gc_instance, Placement(x=780, y=40, rotation=0))
    schematic.add_instance("gc_3", gc_instance, Placement(x=780, y=-40, rotation=0))

    nets = [
        Net(p1="gc_0,o1", p2="mmi_0,o2", name="gc0_to_mmi0_in1"),
        Net(p1="gc_1,o1", p2="mmi_0,o1", name="gc1_to_mmi0_in2"),
        Net(p1="mmi_0,o3", p2="heater_0,o1", name="mmi0_out1_to_heater"),
        Net(p1="heater_0,o2", p2="mmi_1,o2", name="heater_to_mmi1_in1"),
        Net(p1="mmi_0,o4", p2="mmi_1,o1", name="mmi0_out2_to_mmi1_in2"),
        Net(p1="mmi_1,o3", p2="gc_2,o1", name="mmi1_out1_to_gc2"),
        Net(p1="mmi_1,o4", p2="gc_3,o1", name="mmi1_out2_to_gc3"),
    ]
    for net in nets:
        schematic.add_net(net)

    return schematic


if __name__ == "__main__":
    from translation.layout_from_schematic import layout_from_schematic

    schematic = build_schematic()
    print("Schematic instances:", list(schematic.netlist.instances.keys()))
    print("Schematic placements:", list(schematic.placements.keys()))
    print("\nNets defined:")
    for net_name, bundle in schematic.netlist.routes.items():
        print(f"  - {net_name}: {bundle.links}")
    print(f"\nTotal nets: {len(schematic.netlist.routes)}")

    # Build placement-only layout from the schematic and open it in gdsfactory/KLayout.
    layout = layout_from_schematic(schematic)
    print(f"Opening unrouted benchmark layout: {layout.name}")
    layout.show()
