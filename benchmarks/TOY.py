"""TOY benchmark: Simple 2-component schematic for testing routing flow."""

from gdsfactory.generic_tech import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

NODE_TYPES = {
    "gc_0": "input",
    "gc_1": "input",
    "mmi_0": "gate",
    "gc_2": "output",
    "gc_3": "output",
}

INTERNAL_DELAYS_UM = {
    "gc_0": 0.0,
    "gc_1": 0.0,
    "mmi_0": 0.0,
    "gc_2": 0.0,
    "gc_3": 0.0,
}


def build_schematic() -> Schematic:
    """Build a simple TOY schematic with a grating coupler and MMI.

    Topology:
        gc_0     gc_2
        gc_1 -- mmi_0 -- gc_2
               gc_3

    Connectivity:
        gc_0,o2 → mmi_0,o1 (left input 1)
        gc_1,o2 → mmi_0,o2 (left input 2)
        mmi_0,o3 → gc_2,o2 (right output 1)
        mmi_0,o4 → gc_3,o2 (right output 2)
    """
    PDK = get_generic_pdk()
    PDK.activate()

    schematic = Schematic()

    # Create instances
    gc_instance = Instance(component="grating_coupler_te")
    mmi_instance = Instance(component="mmi2x2")

    # Create placements for each instance
    gc_placement0 = Placement(x=0, y=100, mirror=True)
    gc_placement1 = Placement(x=0, y=0, mirror=True)
    gc_placement2 = Placement(x=200, y=100, rotation=0)
    gc_placement3 = Placement(x=200, y=0, rotation=0)
    mmi_placement = Placement(x=100, y=80, rotation=0)

    # Add instances and placements to schematic
    schematic.add_instance("gc_0", gc_instance, gc_placement0)
    schematic.add_instance("gc_1", gc_instance, gc_placement1)
    schematic.add_instance("gc_2", gc_instance, gc_placement2)
    schematic.add_instance("gc_3", gc_instance, gc_placement3)
    schematic.add_instance("mmi_0", mmi_instance, mmi_placement)

    # Define nets (logical connectivity)
    # Left grating couplers → MMI inputs (use optical port 'o1' on grating couplers)
    net_gc0_to_mmi = Net(p1="gc_0,o1", p2="mmi_0,o2", name="gc0_to_mmi_in1")
    net_gc1_to_mmi = Net(p1="gc_1,o1", p2="mmi_0,o1", name="gc1_to_mmi_in2")

    # MMI outputs → Right grating couplers (use optical port 'o1' on grating couplers)
    net_mmi_to_gc2 = Net(p1="mmi_0,o3", p2="gc_2,o1", name="mmi_out1_to_gc2")
    net_mmi_to_gc3 = Net(p1="mmi_0,o4", p2="gc_3,o1", name="mmi_out2_to_gc3")

    # Add nets to schematic
    schematic.add_net(net_gc0_to_mmi)
    schematic.add_net(net_gc1_to_mmi)
    schematic.add_net(net_mmi_to_gc2)
    schematic.add_net(net_mmi_to_gc3)

    return schematic


if __name__ == "__main__":
    # For standalone testing
    schematic = build_schematic()
    print("Schematic instances:", list(schematic.netlist.instances.keys()))
    print("Schematic placements:", list(schematic.placements.keys()))
    print("\nNets defined:")
    for net in schematic.netlist.routes.keys():
        print(f"  - {net}")
    print(f"\nTotal nets: {len(schematic.netlist.routes)}")
