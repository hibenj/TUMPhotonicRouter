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
        node_types[f"heater_output_{lane_idx}"] = "gate"
        node_types[f"mmi_output_{lane_idx}"] = "gate"
        node_types[f"heater_final_{lane_idx}"] = "gate"
        node_types[f"mmi_final_{lane_idx}"] = "gate"
    node_types["mmi_extra_0"] = "gate"
    node_types["heater_extra_0"] = "gate"
    node_types["mmi_extra_1"] = "gate"
    node_types["mmi_extra_2"] = "gate"
    node_types["heater_extra_1"] = "gate"
    node_types["mmi_extra_3"] = "gate"
    node_types["mmi_extra_4"] = "gate"
    node_types["heater_extra_2"] = "gate"
    node_types["mmi_extra_5"] = "gate"
    node_types["mmi_extra_6"] = "gate"
    node_types["heater_extra_3"] = "gate"
    node_types["mmi_extra_7"] = "gate"
    node_types["mmi_extra_8"] = "gate"
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
    extra_mmi2_x = 2200
    extra_heater1_x = 2340
    extra_mmi3_x = 2760
    extra_mmi4_x = 2860
    extra_heater2_x = 3000
    extra_mmi5_x = 3420
    extra_mmi6_x = 3740
    extra_heater3_x = 3880
    extra_mmi7_x = 4300
    extra_mmi8_x = 4780
    output_heater_x = 5180
    output_mmi_x = 5600
    final_heater_x = 5740
    final_mmi_x = 6160
    extra_output_gc_x = 6380
    input_gc_x = 0
    path_offset_y = 40
    lane_centers_y = [240, 80, -80, -240]
    extra_center_y = 140
    extra_mmi2_center_y = 80
    extra_heater1_y = extra_mmi2_center_y - path_offset_y
    extra_mmi4_center_y = 0
    extra_heater2_y = extra_mmi4_center_y + path_offset_y
    extra_mmi6_center_y = extra_mmi4_center_y - 80
    extra_heater3_y = extra_mmi6_center_y - path_offset_y
    extra_mmi8_center_y = extra_mmi6_center_y - 80

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
    schematic.add_instance(
        "mmi_extra_2",
        mmi_instance,
        Placement(x=extra_mmi2_x, y=extra_mmi2_center_y, rotation=0),
    )
    schematic.add_instance(
        "heater_extra_1",
        heater_instance,
        Placement(x=extra_heater1_x, y=extra_heater1_y, rotation=0),
    )
    schematic.add_instance(
        "mmi_extra_3",
        mmi_instance,
        Placement(x=extra_mmi3_x, y=extra_mmi2_center_y, rotation=0),
    )
    schematic.add_instance(
        "mmi_extra_4",
        mmi_instance,
        Placement(x=extra_mmi4_x, y=extra_mmi4_center_y, rotation=0),
    )
    schematic.add_instance(
        "heater_extra_2",
        heater_instance,
        Placement(x=extra_heater2_x, y=extra_heater2_y, rotation=0),
    )
    schematic.add_instance(
        "mmi_extra_5",
        mmi_instance,
        Placement(x=extra_mmi5_x, y=extra_mmi4_center_y, rotation=0),
    )
    schematic.add_instance(
        "mmi_extra_6",
        mmi_instance,
        Placement(x=extra_mmi6_x, y=extra_mmi6_center_y, rotation=0),
    )
    schematic.add_instance(
        "heater_extra_3",
        heater_instance,
        Placement(x=extra_heater3_x, y=extra_heater3_y, rotation=0),
    )
    schematic.add_instance(
        "mmi_extra_7",
        mmi_instance,
        Placement(x=extra_mmi7_x, y=extra_mmi6_center_y, rotation=0),
    )
    schematic.add_instance(
        "mmi_extra_8",
        mmi_instance,
        Placement(x=extra_mmi8_x, y=extra_mmi8_center_y, rotation=0),
    )
    for output_pair_idx, output_center_y in enumerate(lane_centers_y):
        schematic.add_instance(
            f"heater_output_{output_pair_idx}",
            heater_instance,
            Placement(
                x=output_heater_x,
                y=output_center_y + path_offset_y,
                rotation=0,
            ),
        )
        schematic.add_instance(
            f"mmi_output_{output_pair_idx}",
            mmi_instance,
            Placement(x=output_mmi_x, y=output_center_y, rotation=0),
        )
        schematic.add_instance(
            f"heater_final_{output_pair_idx}",
            heater_instance,
            Placement(
                x=final_heater_x,
                y=output_center_y + path_offset_y,
                rotation=0,
            ),
        )
        schematic.add_instance(
            f"mmi_final_{output_pair_idx}",
            mmi_instance,
            Placement(x=final_mmi_x, y=output_center_y, rotation=0),
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
        upper_output_x = extra_output_gc_x
        lower_output_x = extra_output_gc_x
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
        if upper_idx == 0:
            nets.append(
                Net(
                    p1=f"heater_post_{lane_idx},o2",
                    p2="heater_output_0,o1",
                    name="heater_post_0_to_heater_output_0",
                )
            )
        elif upper_idx == 2:
            nets.append(
                Net(
                    p1=f"heater_post_{lane_idx},o2",
                    p2="mmi_extra_0,o1",
                    name="heater_post_1_to_mmi_extra_0_lower_in",
                )
            )
        elif upper_idx == 4:
            nets.append(
                Net(
                    p1=f"heater_post_{lane_idx},o2",
                    p2="mmi_extra_4,o1",
                    name="heater_post_2_to_mmi_extra_4_lower_in",
                )
            )
        elif upper_idx == 6:
            nets.append(
                Net(
                    p1=f"heater_post_{lane_idx},o2",
                    p2="mmi_extra_8,o1",
                    name="heater_post_3_to_mmi_extra_8_lower_in",
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
        elif lower_idx == 3:
            nets.append(
                Net(
                    p1=f"mmi_b_{lane_idx},o4",
                    p2="mmi_extra_2,o1",
                    name="mmi_b_1_lower_to_mmi_extra_2_lower_in",
                )
            )
        elif lower_idx == 5:
            nets.append(
                Net(
                    p1=f"mmi_b_{lane_idx},o4",
                    p2="mmi_extra_6,o1",
                    name="mmi_b_2_lower_to_mmi_extra_6_lower_in",
                )
            )
        elif lower_idx == 7:
            nets.append(
                Net(
                    p1=f"mmi_b_{lane_idx},o4",
                    p2="mmi_output_3,o1",
                    name="mmi_b_3_lower_to_mmi_output_3_lower_in",
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
            p2="mmi_output_0,o1",
            name="mmi_extra_1_upper_to_mmi_output_0_lower_in",
        ),
        Net(
            p1="heater_output_0,o2",
            p2="mmi_output_0,o2",
            name="heater_output_0_to_mmi_output_0_upper_in",
        ),
        Net(
            p1="mmi_output_0,o3",
            p2="heater_final_0,o1",
            name="mmi_output_0_upper_to_heater_final_0",
        ),
        Net(
            p1="mmi_output_0,o4",
            p2="mmi_final_0,o1",
            name="mmi_output_0_lower_to_mmi_final_0_lower_in",
        ),
        Net(
            p1="heater_final_0,o2",
            p2="mmi_final_0,o2",
            name="heater_final_0_to_mmi_final_0_upper_in",
        ),
        Net(
            p1="mmi_final_0,o3",
            p2="gc_out_0,o1",
            name="mmi_final_0_upper_to_gc_out_0",
        ),
        Net(
            p1="mmi_final_0,o4",
            p2="gc_out_1,o1",
            name="mmi_final_0_lower_to_gc_out_1",
        ),
        Net(
            p1="mmi_extra_1,o4",
            p2="mmi_extra_2,o2",
            name="mmi_extra_1_lower_to_mmi_extra_2_upper_in",
        ),
        Net(
            p1="mmi_extra_2,o3",
            p2="mmi_extra_3,o2",
            name="mmi_extra_2_upper_to_mmi_extra_3_upper_in",
        ),
        Net(
            p1="mmi_extra_2,o4",
            p2="heater_extra_1,o1",
            name="mmi_extra_2_lower_to_heater_extra_1",
        ),
        Net(
            p1="heater_extra_1,o2",
            p2="mmi_extra_3,o1",
            name="heater_extra_1_to_mmi_extra_3_lower_in",
        ),
        Net(
            p1="mmi_extra_3,o3",
            p2="heater_output_1,o1",
            name="mmi_extra_3_upper_to_heater_output_1",
        ),
        Net(
            p1="mmi_extra_3,o4",
            p2="mmi_extra_4,o2",
            name="mmi_extra_3_lower_to_mmi_extra_4_upper_in",
        ),
        Net(
            p1="mmi_extra_4,o3",
            p2="heater_extra_2,o1",
            name="mmi_extra_4_upper_to_heater_extra_2",
        ),
        Net(
            p1="heater_extra_2,o2",
            p2="mmi_extra_5,o2",
            name="heater_extra_2_to_mmi_extra_5_upper_in",
        ),
        Net(
            p1="mmi_extra_4,o4",
            p2="mmi_extra_5,o1",
            name="mmi_extra_4_lower_to_mmi_extra_5_lower_in",
        ),
        Net(
            p1="mmi_extra_5,o3",
            p2="mmi_output_1,o1",
            name="mmi_extra_5_upper_to_mmi_output_1_lower_in",
        ),
        Net(
            p1="heater_output_1,o2",
            p2="mmi_output_1,o2",
            name="heater_output_1_to_mmi_output_1_upper_in",
        ),
        Net(
            p1="mmi_output_1,o3",
            p2="heater_final_1,o1",
            name="mmi_output_1_upper_to_heater_final_1",
        ),
        Net(
            p1="mmi_output_1,o4",
            p2="mmi_final_1,o1",
            name="mmi_output_1_lower_to_mmi_final_1_lower_in",
        ),
        Net(
            p1="heater_final_1,o2",
            p2="mmi_final_1,o2",
            name="heater_final_1_to_mmi_final_1_upper_in",
        ),
        Net(
            p1="mmi_final_1,o3",
            p2="gc_out_2,o1",
            name="mmi_final_1_upper_to_gc_out_2",
        ),
        Net(
            p1="mmi_final_1,o4",
            p2="gc_out_3,o1",
            name="mmi_final_1_lower_to_gc_out_3",
        ),
        Net(
            p1="mmi_extra_5,o4",
            p2="mmi_extra_6,o2",
            name="mmi_extra_5_lower_to_mmi_extra_6_upper_in",
        ),
        Net(
            p1="mmi_extra_6,o3",
            p2="mmi_extra_7,o2",
            name="mmi_extra_6_upper_to_mmi_extra_7_upper_in",
        ),
        Net(
            p1="mmi_extra_6,o4",
            p2="heater_extra_3,o1",
            name="mmi_extra_6_lower_to_heater_extra_3",
        ),
        Net(
            p1="heater_extra_3,o2",
            p2="mmi_extra_7,o1",
            name="heater_extra_3_to_mmi_extra_7_lower_in",
        ),
        Net(
            p1="mmi_extra_7,o3",
            p2="heater_output_2,o1",
            name="mmi_extra_7_upper_to_heater_output_2",
        ),
        Net(
            p1="mmi_extra_7,o4",
            p2="mmi_extra_8,o2",
            name="mmi_extra_7_lower_to_mmi_extra_8_upper_in",
        ),
        Net(
            p1="mmi_extra_8,o3",
            p2="mmi_output_2,o1",
            name="mmi_extra_8_upper_to_mmi_output_2_lower_in",
        ),
        Net(
            p1="heater_output_2,o2",
            p2="mmi_output_2,o2",
            name="heater_output_2_to_mmi_output_2_upper_in",
        ),
        Net(
            p1="mmi_output_2,o3",
            p2="heater_final_2,o1",
            name="mmi_output_2_upper_to_heater_final_2",
        ),
        Net(
            p1="mmi_output_2,o4",
            p2="mmi_final_2,o1",
            name="mmi_output_2_lower_to_mmi_final_2_lower_in",
        ),
        Net(
            p1="heater_final_2,o2",
            p2="mmi_final_2,o2",
            name="heater_final_2_to_mmi_final_2_upper_in",
        ),
        Net(
            p1="mmi_final_2,o3",
            p2="gc_out_4,o1",
            name="mmi_final_2_upper_to_gc_out_4",
        ),
        Net(
            p1="mmi_final_2,o4",
            p2="gc_out_5,o1",
            name="mmi_final_2_lower_to_gc_out_5",
        ),
        Net(
            p1="mmi_extra_8,o4",
            p2="heater_output_3,o1",
            name="mmi_extra_8_lower_to_heater_output_3",
        ),
        Net(
            p1="heater_output_3,o2",
            p2="mmi_output_3,o2",
            name="heater_output_3_to_mmi_output_3_upper_in",
        ),
        Net(
            p1="mmi_output_3,o3",
            p2="heater_final_3,o1",
            name="mmi_output_3_upper_to_heater_final_3",
        ),
        Net(
            p1="mmi_output_3,o4",
            p2="mmi_final_3,o1",
            name="mmi_output_3_lower_to_mmi_final_3_lower_in",
        ),
        Net(
            p1="heater_final_3,o2",
            p2="mmi_final_3,o2",
            name="heater_final_3_to_mmi_final_3_upper_in",
        ),
        Net(
            p1="mmi_final_3,o3",
            p2="gc_out_6,o1",
            name="mmi_final_3_upper_to_gc_out_6",
        ),
        Net(
            p1="mmi_final_3,o4",
            p2="gc_out_7,o1",
            name="mmi_final_3_lower_to_gc_out_7",
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
