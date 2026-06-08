"""Eight-input, four-lane MMI-heater-MMI-heater benchmark.

This benchmark is placement-only: it defines instances and logical nets, but
does not route waveguides itself. Each adjacent input pair enters one first
stage MMI. The upper output of each first-stage MMI passes through a heater,
while the lower output bypasses the heater. Both paths then meet again in a
second-stage MMI, whose upper output passes through another heater.
"""

from gdsfactory.generic_tech import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

LANE_COUNT = 4


def _node_types() -> dict[str, str]:
    node_types: dict[str, str] = {}
    for input_idx in range(2 * LANE_COUNT):
        node_types[f"gc_in_{input_idx}"] = "input"
        node_types[f"gc_out_{input_idx}"] = "output"
    for lane_idx in range(LANE_COUNT):
        node_types[f"mmi_a_{lane_idx}"] = "gate"
        node_types[f"heater_{lane_idx}"] = "gate"
        node_types[f"mmi_b_{lane_idx}"] = "gate"
        node_types[f"heater_post_{lane_idx}"] = "gate"
    node_types["mmi_extra_0"] = "gate"
    node_types["heater_extra_0"] = "gate"
    node_types["mmi_extra_1"] = "gate"
    return node_types


NODE_TYPES = _node_types()

INTERNAL_DELAYS_UM = {
    instance_name: ("auto" if instance_name.startswith("heater_") else 0.0)
    for instance_name in NODE_TYPES
}


def build_schematic() -> Schematic:
    """Build the 8-input / 4-lane MMI-heater-MMI-heater benchmark.

    Topology per lane:
        gc_in_even  -> mmi_a.o2 -> heater -> mmi_b.o2 -> heater_post -> gc_out_even
        gc_in_odd   -> mmi_a.o1 ----------> mmi_b.o1 ---------------> gc_out_odd

    The even-numbered inputs are the upper paths in each lane, preserving the
    input order across all four lanes.
    """
    pdk = get_generic_pdk()
    pdk.activate()

    schematic = Schematic()

    gc_instance = Instance(component="grating_coupler_te")
    mmi_instance = Instance(component="mmi2x2")
    heater_instance = Instance(component="straight_heater_metal")

    first_mmi_x = 160
    heater_x = 300
    second_mmi_x = 720
    post_heater_x = 820
    output_gc_x = 1220
    extra_mmi0_x = 1340
    extra_heater_x = 1480
    extra_mmi1_x = 1900
    extra_output_gc_x = 2080
    input_gc_x = 0
    path_offset_y = 40
    lane_centers_y = [240, 80, -80, -240]
    extra_center_y = 140

    schematic.add_instance(
        "mmi_extra_0",
        mmi_instance,
        Placement(x=extra_mmi0_x, y=extra_center_y, rotation=0),
    )
    schematic.add_instance(
        "heater_extra_0",
        heater_instance,
        Placement(x=extra_heater_x, y=extra_center_y + path_offset_y, rotation=0),
    )
    schematic.add_instance(
        "mmi_extra_1",
        mmi_instance,
        Placement(x=extra_mmi1_x, y=extra_center_y, rotation=0),
    )

    for lane_idx, center_y in enumerate(lane_centers_y):
        upper_idx = 2 * lane_idx
        lower_idx = upper_idx + 1

        schematic.add_instance(
            f"gc_in_{upper_idx}",
            gc_instance,
            Placement(x=input_gc_x, y=center_y + path_offset_y, mirror=True),
        )
        schematic.add_instance(
            f"gc_in_{lower_idx}",
            gc_instance,
            Placement(x=input_gc_x, y=center_y - path_offset_y, mirror=True),
        )
        schematic.add_instance(
            f"mmi_a_{lane_idx}",
            mmi_instance,
            Placement(x=first_mmi_x, y=center_y, rotation=0),
        )
        schematic.add_instance(
            f"heater_{lane_idx}",
            heater_instance,
            Placement(x=heater_x, y=center_y + path_offset_y, rotation=0),
        )
        schematic.add_instance(
            f"mmi_b_{lane_idx}",
            mmi_instance,
            Placement(x=second_mmi_x, y=center_y, rotation=0),
        )
        schematic.add_instance(
            f"heater_post_{lane_idx}",
            heater_instance,
            Placement(x=post_heater_x, y=center_y + path_offset_y, rotation=0),
        )
        upper_output_x = extra_output_gc_x if upper_idx == 2 else output_gc_x
        lower_output_x = extra_output_gc_x if lower_idx == 1 else output_gc_x
        schematic.add_instance(
            f"gc_out_{upper_idx}",
            gc_instance,
            Placement(x=upper_output_x, y=center_y + path_offset_y, rotation=0),
        )
        schematic.add_instance(
            f"gc_out_{lower_idx}",
            gc_instance,
            Placement(x=lower_output_x, y=center_y - path_offset_y, rotation=0),
        )

        nets = [
            Net(
                p1=f"gc_in_{upper_idx},o1",
                p2=f"mmi_a_{lane_idx},o2",
                name=f"gc_in_{upper_idx}_to_mmi_a_{lane_idx}_upper_in",
            ),
            Net(
                p1=f"gc_in_{lower_idx},o1",
                p2=f"mmi_a_{lane_idx},o1",
                name=f"gc_in_{lower_idx}_to_mmi_a_{lane_idx}_lower_in",
            ),
            Net(
                p1=f"mmi_a_{lane_idx},o3",
                p2=f"heater_{lane_idx},o1",
                name=f"mmi_a_{lane_idx}_upper_to_heater_{lane_idx}",
            ),
            Net(
                p1=f"heater_{lane_idx},o2",
                p2=f"mmi_b_{lane_idx},o2",
                name=f"heater_{lane_idx}_to_mmi_b_{lane_idx}_upper_in",
            ),
            Net(
                p1=f"mmi_a_{lane_idx},o4",
                p2=f"mmi_b_{lane_idx},o1",
                name=f"mmi_a_{lane_idx}_lower_to_mmi_b_{lane_idx}_lower_in",
            ),
            Net(
                p1=f"mmi_b_{lane_idx},o3",
                p2=f"heater_post_{lane_idx},o1",
                name=f"mmi_b_{lane_idx}_upper_to_heater_post_{lane_idx}",
            ),
        ]
        if upper_idx == 2:
            nets.append(
                Net(
                    p1=f"heater_post_{lane_idx},o2",
                    p2="mmi_extra_0,o1",
                    name="heater_post_1_to_mmi_extra_0_lower_in",
                )
            )
        else:
            nets.append(
                Net(
                    p1=f"heater_post_{lane_idx},o2",
                    p2=f"gc_out_{upper_idx},o1",
                    name=f"heater_post_{lane_idx}_to_gc_out_{upper_idx}",
                )
            )
        if lower_idx == 1:
            nets.append(
                Net(
                    p1=f"mmi_b_{lane_idx},o4",
                    p2="mmi_extra_0,o2",
                    name="mmi_b_0_lower_to_mmi_extra_0_upper_in",
                )
            )
        else:
            nets.append(
                Net(
                    p1=f"mmi_b_{lane_idx},o4",
                    p2=f"gc_out_{lower_idx},o1",
                    name=f"mmi_b_{lane_idx}_lower_to_gc_out_{lower_idx}",
                )
            )
        for net in nets:
            schematic.add_net(net)

    extra_nets = [
        Net(
            p1="mmi_extra_0,o3",
            p2="heater_extra_0,o1",
            name="mmi_extra_0_upper_to_heater_extra_0",
        ),
        Net(
            p1="heater_extra_0,o2",
            p2="mmi_extra_1,o2",
            name="heater_extra_0_to_mmi_extra_1_upper_in",
        ),
        Net(
            p1="mmi_extra_0,o4",
            p2="mmi_extra_1,o1",
            name="mmi_extra_0_lower_to_mmi_extra_1_lower_in",
        ),
        Net(
            p1="mmi_extra_1,o3",
            p2="gc_out_1,o1",
            name="mmi_extra_1_upper_to_gc_out_1",
        ),
        Net(
            p1="mmi_extra_1,o4",
            p2="gc_out_2,o1",
            name="mmi_extra_1_lower_to_gc_out_2",
        ),
    ]
    for net in extra_nets:
        schematic.add_net(net)

    return schematic


if __name__ == "__main__":
    from pathlib import Path
    import sys

    repo_root = Path(__file__).resolve().parents[1]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from translation.layout_from_schematic import layout_from_schematic

    schematic = build_schematic()
    print("Schematic instances:", list(schematic.netlist.instances.keys()))
    print("Schematic placements:", list(schematic.placements.keys()))
    print("\nNets defined:")
    for net_name, bundle in schematic.netlist.routes.items():
        print(f"  - {net_name}: {bundle.links}")
    print(f"\nTotal nets: {len(schematic.netlist.routes)}")

    layout = layout_from_schematic(schematic)
    print(f"Opening unrouted benchmark layout in KLayout: {layout.name}")
    layout.show()
