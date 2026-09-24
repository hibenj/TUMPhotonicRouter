"""Twenty heaters in four lanes: the wide electrical benchmark.

This is the large case of the electrical benchmark suite
(`scripts/benchmark_electrical.py`): twenty heaters on a 6.5 mm die, so twenty
heater terminal pairs, twenty detailed routes and the widest pad row the suite
runs. It replaces the deleted toy module `mmi_heater_8x4` (Milestone 8, Slice D
of `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`) and
keeps that module's heater positions, which is why the electrical metrics of the
case did not move when the suite switched to it.

The optical part is trivial because there is none: this benchmark places heaters
only -- no MMIs, no grating couplers, no optical nets. What the electrical
router reads from a schematic is only `netlist.instances` (instance name ->
component name, so it can tell which instances are heaters); everything else it
needs comes from the built layout's geometry. The one non-heater instance is the
die outline of `MULTI_HEATER_DIE_UM`, drawn on the waveguide layer so it sets
the die extent without blocking any electrical route.

`benchmarks/heater_lanes_ripup.py` is the third case of the suite and builds on
`heater_only_schematic` below.
"""

from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Placement, Schematic

HEATER_COMPONENT = "straight_heater_metal"

# The pairs that share a row -- (`heater_N`, `heater_post_N`) and
# (`heater_output_N`, `heater_final_N`) for each lane N, plus
# (`heater_extra_1`, `heater_extra_2`) -- are what the common-bus local-pair
# selection and local-trunk strategies are about, so the names and the row
# sharing are part of this benchmark's contract (`tests/test_electrical_routing.py`
# takes the same placement through `tests/fixtures/synthetic_layouts.py`).
MULTI_HEATER_PLACEMENTS_UM: dict[str, tuple[float, float]] = {
    "heater_0": (300.0, 280.0),
    "heater_post_0": (820.0, 280.0),
    "heater_1": (300.0, 120.0),
    "heater_post_1": (820.0, 120.0),
    "heater_2": (300.0, -40.0),
    "heater_post_2": (820.0, -40.0),
    "heater_3": (300.0, -200.0),
    "heater_post_3": (820.0, -200.0),
    "heater_extra_0": (1480.0, 180.0),
    "heater_extra_1": (2340.0, 40.0),
    "heater_extra_2": (3000.0, 40.0),
    "heater_extra_3": (3880.0, -120.0),
    "heater_output_0": (5180.0, 280.0),
    "heater_final_0": (5740.0, 280.0),
    "heater_output_1": (5180.0, 120.0),
    "heater_final_1": (5740.0, 120.0),
    "heater_output_2": (5180.0, -40.0),
    "heater_final_2": (5740.0, -40.0),
    "heater_output_3": (5180.0, -200.0),
    "heater_final_3": (5740.0, -200.0),
}

# The die outline of a heater-only layout, as (x0, y0, width, height) in
# micrometres. The electrical router lays its pad row out above the layout's
# bounding box and its common-bus rail below it, so a heater-only layout needs a
# die to work against or the pad row lands on top of the heaters and the escape
# corridors collide (verification then reports `metal_overlaps_raw_obstacle`).
# The outline is drawn on the waveguide layer (1, 0), which is NOT one of the
# electrical obstacle layers (`ElectricalRoutingConfig.metal_obstacle_layers`
# covers metal and heater layers only), so it sets the die extent without
# blocking any electrical route. The die is 600 um tall, which clears the heater
# rows (y -200 to 280 plus the heater's own height) with room to spare: at 560 um
# the escapes of the outer lanes have nowhere to go and the verifier reports
# cross-net overlaps, while every height from 580 um up leaves the cross-net
# spacing at 20 um, twice the 10 um the verifier requires.
MULTI_HEATER_DIE_UM = (-50.0, -300.0, 6500.0, 600.0)
DIE_OUTLINE_LAYER = (1, 0)

NODE_TYPES = dict.fromkeys(MULTI_HEATER_PLACEMENTS_UM, "gate")

# Resolved from straight_heater_metal.info.length at metadata load time. The die
# outline is not a routing node and carries no optical path, so it is in neither
# table.
INTERNAL_DELAYS_UM: dict[str, object] = dict.fromkeys(MULTI_HEATER_PLACEMENTS_UM, "auto")


def heater_only_schematic(
    placements_um: dict[str, tuple[float, float]],
    die_um: tuple[float, float, float, float],
) -> Schematic:
    """Heaters at the given positions on a `die_um` waveguide-layer die.

    The electrical router needs no optical nets (it reads only which instances
    are heaters, then works on the built layout's metal geometry), so this
    builder adds none: the layout is the heaters' metal plus the die outline.
    """

    pdk = get_generic_pdk()
    pdk.activate()

    schematic = Schematic()
    heater = Instance(component=HEATER_COMPONENT)
    for instance_name, (x_um, y_um) in placements_um.items():
        schematic.add_instance(instance_name, heater, Placement(x=x_um, y=y_um, rotation=0))
    die_x_um, die_y_um, die_width_um, die_height_um = die_um
    schematic.add_instance(
        "die_outline",
        Instance(
            component="rectangle",
            settings={"size": (die_width_um, die_height_um), "layer": DIE_OUTLINE_LAYER},
        ),
        Placement(x=die_x_um, y=die_y_um),
    )
    return schematic


def build_schematic() -> Schematic:
    """Twenty heaters in four lanes; see `MULTI_HEATER_PLACEMENTS_UM`."""

    return heater_only_schematic(MULTI_HEATER_PLACEMENTS_UM, MULTI_HEATER_DIE_UM)
