"""Synthetic layout/schematic/component builders shared across the test suite.

ExecPlan Milestone 6, Slice 2: these builders used to be defined once per test
file (`tests/test_layout_from_schematic.py`, `tests/test_route_rust_opened_cells.py`,
`tests/test_routing_stages.py`, `tests/test_static_obstacle_builder.py`); this
module is now the one place synthetic layouts are built, so a test file either
imports the builder it needs or takes it through the `tests/conftest.py`
fixtures. No behaviour changed in the move: every function and dataclass in the
first four sections is the same code that used to live in the test file named
in its section.

Slice 3 added the sections after those: the two-instance S-bend layout, the
path-length matching schematic and the heater layouts for the electrical
router. Those are new code, written so that no test has to import one of the
toy benchmarks (`TOY`, `mmi_heater`, `mmi_heater_8x4`,
`mmi_heater_8x4_ripup_reroute`) that Milestone 8 may delete.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

from photonic_router.static_obstacle_builder import GridSpec

get_generic_pdk().activate()

# --- single-rectangle schematic (tests/test_layout_from_schematic.py) ------


def single_rectangle_schematic(placement: Placement) -> Schematic:
    """A one-instance schematic: a 10x2 `rectangle` at `placement`."""

    schematic = Schematic()
    schematic.add_instance(
        "rect_0",
        Instance(component="rectangle", settings={"size": (10, 2)}),
        placement,
    )
    return schematic


# --- dummy schematic (tests/test_route_rust_opened_cells.py) --------------


@dataclass
class DummyBundle:
    links: dict[str, str]


@dataclass
class DummyNetlist:
    routes: dict[str, DummyBundle]
    instances: dict[str, Any] = field(default_factory=dict)


@dataclass
class DummySchematic:
    netlist: DummyNetlist


def make_dummy_layout() -> Component:
    """A fresh, uniquely-named empty layout for a `route_nets_rust` call."""

    return Component(f"dummy_layout_{uuid4().hex}")


# --- three-net pipeline layout (tests/test_routing_stages.py) -------------

GRID_WIDTH = 30
GRID_HEIGHT = 20

# port spec -> (centre in micrometres, orientation in degrees).
PORTS: dict[str, tuple[tuple[float, float], float]] = {
    "left,o1": ((2.5, 4.5), 0.0),
    "left,o2": ((2.5, 9.5), 0.0),
    "left,o3": ((2.5, 14.5), 0.0),
    "right0,o1": ((27.5, 4.5), 180.0),
    "right1,o1": ((27.5, 9.5), 180.0),
    "right2,o1": ((27.5, 14.5), 180.0),
}
LINKS = {
    "net_0": ("left,o1", "right0,o1"),
    "net_1": ("left,o2", "right1,o1"),
    "net_2": ("left,o3", "right2,o1"),
}


class ObstacleMapStandIn:
    """What phase 1 expects of `build_static_obstacle_map`: a grid and no obstacle."""

    def __init__(self) -> None:
        self.grid = GridSpec(
            width=GRID_WIDTH,
            height=GRID_HEIGHT,
            grid_size_um=1.0,
            origin=(0.0, 0.0),
            die_bbox=(0.0, 0.0, float(GRID_WIDTH), float(GRID_HEIGHT)),
        )
        self.blocked_cells: set[tuple[int, int]] = set()
        self.raw_blocked_cells: set[tuple[int, int]] = set()
        self.port_open_cells: set[tuple[int, int]] = set()

    def export_debug_svg(self, path: Any) -> None:  # pragma: no cover - debug only
        path.write_text("<svg/>", encoding="utf-8")


def three_net_schematic() -> Any:
    """A `left` instance fanning out to three `right*` grating couplers."""

    return SimpleNamespace(
        netlist=SimpleNamespace(
            routes={
                name: SimpleNamespace(links={source: target})
                for name, (source, target) in LINKS.items()
            },
            instances={
                "left": SimpleNamespace(component="left_mmi"),
                "right0": SimpleNamespace(component="right_gc"),
                "right1": SimpleNamespace(component="right_gc"),
                "right2": SimpleNamespace(component="right_gc"),
            },
        )
    )


def port_from_instance(_layout: Any, instance: str, port: str) -> SimpleNamespace:
    center, orientation = PORTS[f"{instance},{port}"]
    return SimpleNamespace(center=center, orientation=orientation)


# --- shared gf.Component builders (tests/test_static_obstacle_builder.py) -


def component_with_square_port_obstacle(name: str) -> Component:
    """A 2x2 square obstacle on layer (1, 0) with port `o1` at its centre.

    Built identically by three `test_static_obstacle_builder.py` tests
    (bounding-box vs. rasterized mode, port cells opened vs. kept blocked);
    only the component name varied between them, so it is now a parameter.
    """

    component = Component(name)
    component.add_polygon(
        [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)],
        layer=(1, 0),
    )
    component.add_port(name="o1", center=(1.5, 1.5), width=0.5, orientation=0.0, layer=(1, 0))
    return component


# --- two-instance S-bend layout (tests/test_route_rust_geometry.py) --------

# The lateral offset of the tightest S-bend the router can build out of two
# 90-degree arcs at a 10 um bend radius on the default 0.5 um grid: the two
# arcs consume 10 um of lateral travel each, so at 20.0 um there is no room
# left for a straight between them (one cell less, 19.0 um, does not route at
# all with 45-degree turns disabled). This is the geometry the deleted
# `test_toy_ten_um_bend_radius_does_not_backtrack_on_one_cell_short_s_bend`
# covered through the TOY toy benchmark (Milestone 0 note of
# `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`).
TIGHT_S_BEND_OFFSET_UM = 20.0
TIGHT_S_BEND_BEND_RADIUS_UM = 10.0
TIGHT_S_BEND_SPAN_UM = 200.0


def two_instance_s_bend_schematic(
    lateral_offset_um: float = TIGHT_S_BEND_OFFSET_UM,
    span_um: float = TIGHT_S_BEND_SPAN_UM,
) -> Schematic:
    """Two straights `span_um` apart in x and `lateral_offset_um` apart in y,
    linked `left,o2 -> right,o1`, so the only route between them is an S-bend.

    Both ports face along +x/-x, so the router has to turn out of the source
    heading and back into the target heading -- an S-bend whose lateral room
    is exactly `lateral_offset_um`.
    """

    schematic = Schematic()
    straight = Instance(component="straight", settings={"length": 20.0})
    schematic.add_instance("left", straight, Placement(x=0.0, y=0.0))
    schematic.add_instance("right", straight, Placement(x=span_um, y=lateral_offset_um))
    schematic.add_net(Net(p1="left,o2", p2="right,o1", name="left_to_right"))
    return schematic


# --- path-length matching schematic (tests/test_path_length_graph.py) ------

# The path-length tests need the smallest schematic that carries a matching
# requirement: two inputs entering one gate (so their arrival lengths must
# match) and two outputs leaving it (so their departure lengths must match).
# Node types and internal delays are what `load_benchmark_metadata` returns
# for a benchmark module, without a benchmark module: every instance is a
# passive coupler or splitter, so no internal delay is inferred.
PATH_LENGTH_NODE_TYPES: dict[str, str] = {
    "gc_0": "input",
    "gc_1": "input",
    "mmi_0": "gate",
    "gc_2": "output",
    "gc_3": "output",
}
PATH_LENGTH_INTERNAL_DELAYS_UM: dict[str, float] = {
    instance_name: 0.0 for instance_name in PATH_LENGTH_NODE_TYPES
}


def path_length_schematic() -> Schematic:
    """Two input grating couplers into one 2x2 MMI and two outputs out of it.

    The four nets are named after their endpoints (`gc0_to_mmi_in1`,
    `gc1_to_mmi_in2`, `mmi_out1_to_gc2`, `mmi_out2_to_gc3`) because the
    path-length tests address records and edges by net name. The placement
    offsets the MMI in y so the two input nets differ in length -- which is
    what makes the input group's matching requirement non-trivial.
    """

    schematic = Schematic()
    grating_coupler = Instance(component="grating_coupler_te")
    mmi = Instance(component="mmi2x2")

    schematic.add_instance("gc_0", grating_coupler, Placement(x=0, y=100, mirror=True))
    schematic.add_instance("gc_1", grating_coupler, Placement(x=0, y=0, mirror=True))
    schematic.add_instance("gc_2", grating_coupler, Placement(x=200, y=100, rotation=0))
    schematic.add_instance("gc_3", grating_coupler, Placement(x=200, y=0, rotation=0))
    schematic.add_instance("mmi_0", mmi, Placement(x=100, y=20, rotation=0))

    for net in (
        Net(p1="gc_0,o1", p2="mmi_0,o2", name="gc0_to_mmi_in1"),
        Net(p1="gc_1,o1", p2="mmi_0,o1", name="gc1_to_mmi_in2"),
        Net(p1="mmi_0,o3", p2="gc_2,o1", name="mmi_out1_to_gc2"),
        Net(p1="mmi_0,o4", p2="gc_3,o1", name="mmi_out2_to_gc3"),
    ):
        schematic.add_net(net)

    return schematic


# --- heater layouts for the electrical router (tests/test_electrical_routing.py) ---

# `tests/test_electrical_routing.py` used to take its layouts from the toy
# benchmarks `mmi_heater`, `mmi_heater_8x4` and `mmi_heater_8x4_ripup_reroute`,
# which Milestone 8 of
# `.agent/execplans/2026-09-22-modular-readable-router-restructure.md` may
# delete. The builders below are the synthetic replacements (Milestone 6,
# Slice 3). What the electrical router reads from a schematic is only
# `netlist.instances` (instance name -> component name, so it can tell which
# instances are heaters); everything else it needs comes from the built
# layout's geometry. So the multi-heater builders place heaters only -- no
# MMIs, no grating couplers and no optical nets -- while the single-heater
# builder keeps the whole seven-instance arrangement because it is small
# enough to state outright.

HEATER_COMPONENT = "straight_heater_metal"


def single_heater_schematic() -> Schematic:
    """One heater between two 2x2 MMIs, with two input and two output grating
    couplers: the smallest die that carries exactly one heater terminal pair.

    The placement is deliberately generous in x so the electrical router has a
    free channel above and below the optical row.
    """

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


# Twenty heaters in four lanes. The pairs that share a row -- (`heater_N`,
# `heater_post_N`) and (`heater_output_N`, `heater_final_N`) for each lane N,
# plus (`heater_extra_1`, `heater_extra_2`) -- are what the common-bus
# local-pair selection and local-trunk strategies are about, so the names and
# the row sharing are part of the fixture's contract.
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

# Eleven heaters: the four lanes' pairs plus three extras staggered in x and
# y, so the individual escapes group into several topology bundles of which
# one (four tracks) is the widest -- what
# `test_auto_pad_channel_height_uses_widest_topology_bundle` compares against
# the eleven-track channel a global assignment would need. The extras sit at
# distinct x positions on purpose: two extras stacked in one vertical corridor
# (the arrangement the mmi_heater_8x4_ripup_reroute toy benchmark happens to
# have)
# leaves their escapes 9.5 um apart, half a micron inside the 10 um cross-net
# clearance, and the verifier rejects the result.
RIPUP_REROUTE_HEATER_PLACEMENTS_UM: dict[str, tuple[float, float]] = {
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
    "heater_extra_2": (3000.0, 0.0),
}


# Die outlines for the two heater-only layouts, as (x0, y0, width, height) in
# micrometres. The electrical router lays its pad row out above the layout's
# bounding box and its common-bus rail below it, so a heater-only layout needs
# a die to work against or the pad row lands on top of the heaters and the
# escape corridors collide (verification then reports
# `metal_overlaps_raw_obstacle`). The outline is drawn on the waveguide layer
# (1, 0), which is NOT one of the electrical obstacle layers
# (`ElectricalRoutingConfig.metal_obstacle_layers` covers metal and heater
# layers only), so it sets the die extent without blocking any electrical
# route. Both dies are 600 um tall, which clears the heater rows (y -200 to
# 280 plus the heater's own height) with room to spare: at 560 um the escapes
# of the outer lanes have nowhere to go and the verifier reports cross-net
# overlaps, while every height from 580 um up leaves the cross-net spacing at
# 20 um, twice the 10 um the verifier requires.
MULTI_HEATER_DIE_UM = (-50.0, -300.0, 6500.0, 600.0)
RIPUP_REROUTE_HEATER_DIE_UM = (-50.0, -300.0, 3750.0, 600.0)
DIE_OUTLINE_LAYER = (1, 0)


def heater_only_schematic(
    placements_um: dict[str, tuple[float, float]],
    die_um: tuple[float, float, float, float],
) -> Schematic:
    """Heaters at the given positions on a `die_um` waveguide-layer die.

    The electrical router needs no optical nets (it reads only which instances
    are heaters, then works on the built layout's metal geometry), so this
    builder adds none: the layout is the heaters' metal plus the die outline.
    """

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


def multi_heater_schematic() -> Schematic:
    """Twenty heaters in four lanes; see `MULTI_HEATER_PLACEMENTS_UM`."""

    return heater_only_schematic(MULTI_HEATER_PLACEMENTS_UM, MULTI_HEATER_DIE_UM)


def ripup_reroute_heater_schematic() -> Schematic:
    """Eleven heaters, two of them in adjacent rows; see
    `RIPUP_REROUTE_HEATER_PLACEMENTS_UM`."""

    return heater_only_schematic(
        RIPUP_REROUTE_HEATER_PLACEMENTS_UM, RIPUP_REROUTE_HEATER_DIE_UM
    )
