import gdsfactory as gf
from gdsfactory.gpdk import get_generic_pdk

from gdsfactory.schematic import (
    Instance,
    Placement,
    Schematic,
)

PDK = get_generic_pdk()
PDK.activate()

if __name__ == "__main__":
    # --- Generate, Plot, and Save the Network ---
    print("Generating TOY...")

    # Create a Schematic instead of a Component
    schematic = Schematic()

    die_size = [1600, 800]

    gc_type = gf.components.grating_coupler_te()
    mmi_type = gf.components.mmi2x2()

    # mmi_type.pprint_ports()
    # gc_type.pprint_ports()

    # Create instances
    gc_instance = Instance(component="grating_coupler_te")
    mmi_instance = Instance(component="mmi2x2")

    # Create placements for each instance
    gc_placement = Placement(x=100, y=100, rotation=0)
    mmi_placement = Placement(x=400, y=200, rotation=90)

    # Add instances and placements to schematic
    schematic.add_instance("gc_0", gc_instance, gc_placement)
    schematic.add_instance("mmi_0", mmi_instance, mmi_placement)

    print("Schematic instances:", schematic.netlist.instances)
    print("Schematic placements:", schematic.placements)
