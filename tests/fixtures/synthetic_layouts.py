"""Synthetic layout/schematic/component builders shared across the test suite.

ExecPlan Milestone 6, Slice 2: these builders used to be defined once per test
file (`tests/test_layout_from_schematic.py`, `tests/test_route_rust_opened_cells.py`,
`tests/test_routing_stages.py`, `tests/test_static_obstacle_builder.py`); this
module is now the one place synthetic layouts are built, so a test file either
imports the builder it needs or takes it through the `tests/conftest.py`
fixtures. No behaviour changed in the move: every function and dataclass below
is the same code that used to live in the test file named in its section.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Placement, Schematic

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
