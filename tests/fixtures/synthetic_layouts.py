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
four toy benchmarks (`TOY`, `mmi_heater`, `mmi_heater_8x4`,
`mmi_heater_8x4_ripup_reroute`) that Milestone 8, Slice D deleted. The heater
layouts became benchmark modules of their own in that slice (`heater_single`,
`heater_lanes_20`, `heater_lanes_ripup`), so the last section only wraps them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

from benchmarks import heater_lanes_20, heater_lanes_ripup, heater_single
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
# which Milestone 6, Slice 3 replaced with synthetic builders here and Milestone
# 8, Slice D of
# `.agent/execplans/2026-09-22-modular-readable-router-restructure.md` deleted.
# Those synthetic layouts are now the benchmark modules `heater_single`,
# `heater_lanes_20` and `heater_lanes_ripup` -- the three cases of
# `scripts/benchmark_electrical.py` -- so the placements, the die outlines and
# the reasoning behind them live there, in one place, and this section only names
# them for the tests. `benchmarks/heater_lanes_20.py::heater_only_schematic` is
# the shared builder of the two heater-only layouts.


def single_heater_schematic() -> Schematic:
    """One heater between two 2x2 MMIs; see `benchmarks/heater_single.py`."""

    return heater_single.build_schematic()


def multi_heater_schematic() -> Schematic:
    """Twenty heaters in four lanes; see `benchmarks/heater_lanes_20.py`."""

    return heater_lanes_20.build_schematic()


def ripup_reroute_heater_schematic() -> Schematic:
    """Eleven heaters, two of them one row apart; see
    `benchmarks/heater_lanes_ripup.py`."""

    return heater_lanes_ripup.build_schematic()
