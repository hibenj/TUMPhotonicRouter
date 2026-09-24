"""One heater between two 2x2 MMIs: the smallest electrical benchmark.

This is the small case of the electrical benchmark suite
(`scripts/benchmark_electrical.py`): exactly one heater, so exactly one heater
terminal pair, one pad-row assignment and one detailed route. It replaces the
deleted toy module `mmi_heater` (Milestone 8, Slice D of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`) and keeps
that module's placement unchanged, which is why the electrical metrics of the
case did not move when the suite switched to it.

The optical part is trivial and nothing here routes it: the two input grating
couplers, the two MMIs and the two output grating couplers exist only to give
the heater a plausible home, and the electrical router reads nothing from them
(it needs `netlist.instances` to see which instances are heaters, and takes
everything else from the built layout's metal geometry). The placement is
deliberately generous in x so the electrical router has a free channel above
and below the optical row.
"""

from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

HEATER_COMPONENT = "straight_heater_metal"

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
    # Resolved from straight_heater_metal.info.length at metadata load time.
    "heater_0": "auto",
    "mmi_1": 0.0,
    "gc_2": 0.0,
    "gc_3": 0.0,
}


def build_schematic() -> Schematic:
    """Build the single-heater benchmark: `gc_0/gc_1 -> mmi_0 -> heater_0 ->
    mmi_1 -> gc_2/gc_3`, placed left to right with generous spacing."""

    pdk = get_generic_pdk()
    pdk.activate()

    schematic = Schematic()

    grating_coupler = Instance(component="grating_coupler_te")
    mmi = Instance(component="mmi2x2")
    heater = Instance(component=HEATER_COMPONENT)

    schematic.add_instance("gc_0", grating_coupler, Placement(x=0, y=40, mirror=True))
    schematic.add_instance("gc_1", grating_coupler, Placement(x=0, y=-40, mirror=True))
    schematic.add_instance("mmi_0", mmi, Placement(x=120, y=0, rotation=0))
    schematic.add_instance("heater_0", heater, Placement(x=240, y=40, rotation=0))
    schematic.add_instance("mmi_1", mmi, Placement(x=620, y=0, rotation=0))
    schematic.add_instance("gc_2", grating_coupler, Placement(x=780, y=40, rotation=0))
    schematic.add_instance("gc_3", grating_coupler, Placement(x=780, y=-40, rotation=0))

    for net in (
        Net(p1="gc_0,o1", p2="mmi_0,o2", name="gc0_to_mmi0_in1"),
        Net(p1="gc_1,o1", p2="mmi_0,o1", name="gc1_to_mmi0_in2"),
        Net(p1="mmi_0,o3", p2="heater_0,o1", name="mmi0_out1_to_heater"),
        Net(p1="heater_0,o2", p2="mmi_1,o2", name="heater_to_mmi1_in1"),
        Net(p1="mmi_0,o4", p2="mmi_1,o1", name="mmi0_out2_to_mmi1_in2"),
        Net(p1="mmi_1,o3", p2="gc_2,o1", name="mmi1_out1_to_gc2"),
        Net(p1="mmi_1,o4", p2="gc_3,o1", name="mmi1_out2_to_gc3"),
    ):
        schematic.add_net(net)

    return schematic
