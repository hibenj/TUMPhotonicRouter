"""LiDAR multiport MMI 8x8 benchmark.

This benchmark mirrors the LiDAR ``multiportmmi_8x8`` netlist and placement
YAML so both routers can be run against the same topology.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic

BENCHMARK_YAML = Path(__file__).with_name("data") / "multiportmmi_8x8.yml"

N = 8

NODE_TYPES: dict[str, str] = {}
INTERNAL_DELAYS_UM: dict[str, float] = {}


def _load_lidar_yaml() -> dict[str, Any]:
    if not BENCHMARK_YAML.exists():
        raise FileNotFoundError(f"Benchmark YAML not found: {BENCHMARK_YAML}")
    return yaml.safe_load(BENCHMARK_YAML.read_text())


def _clean_instance_settings(settings: dict[str, Any]) -> dict[str, Any]:
    cleaned = dict(settings)
    cleaned.pop("macro_type", None)
    cleaned.pop("placement", None)
    cleaned.pop("info", None)
    return cleaned


def _placement_from_lidar(raw: dict[str, Any]) -> Placement:
    return Placement(
        x=raw.get("x"),
        y=raw.get("y"),
        xmin=raw.get("xmin"),
        ymin=raw.get("ymin"),
        xmax=raw.get("xmax"),
        ymax=raw.get("ymax"),
        dx=float(raw.get("dx", 0) or 0),
        dy=float(raw.get("dy", 0) or 0),
        port=raw.get("port"),
        rotation=float(raw.get("rotation", 0) or 0),
        mirror=raw.get("mirror", False),
    )


def _classify_node(instance_name: str, component: str) -> str:
    if instance_name == "gc1":
        return "input"
    if instance_name.startswith("gc_array_out_"):
        return "output"
    if component in {"mmi", "mmi1x2", "mzi"}:
        return "gate"
    return "delay"


def build_schematic() -> Schematic:
    """Build the LiDAR multiport MMI 8x8 benchmark schematic."""
    get_generic_pdk().activate()

    data = _load_lidar_yaml()
    schematic = Schematic()

    node_types: dict[str, str] = {}
    internal_delays: dict[str, float] = {}

    placements = data.get("schematic_placements", {})
    for instance_name, raw_instance in data.get("instances", {}).items():
        component = str(raw_instance["component"])
        settings = _clean_instance_settings(dict(raw_instance.get("settings", {})))
        placement_data = placements.get(instance_name)
        if placement_data is None:
            raise ValueError(f"No placement found for LiDAR instance {instance_name!r}")

        schematic.add_instance(
            instance_name,
            Instance(component=component, settings=settings),
            _placement_from_lidar(dict(placement_data)),
        )
        node_types[instance_name] = _classify_node(instance_name, component)
        internal_delays[instance_name] = 0.0

    for net_name, endpoints in data.get("nets", {}).items():
        if len(endpoints) != 2:
            raise ValueError(f"Expected two endpoints for net {net_name!r}, got {endpoints!r}")
        schematic.add_net(Net(p1=str(endpoints[0]), p2=str(endpoints[1]), name=str(net_name)))

    NODE_TYPES.clear()
    NODE_TYPES.update(node_types)
    INTERNAL_DELAYS_UM.clear()
    INTERNAL_DELAYS_UM.update(internal_delays)

    return schematic


if __name__ == "__main__":
    from translation.layout_from_schematic import layout_from_schematic

    schematic = build_schematic()
    print("Schematic instances:", len(schematic.netlist.instances))
    print("Schematic placements:", len(schematic.placements))
    print("Total nets:", len(schematic.netlist.routes))

    layout = layout_from_schematic(schematic)
    print(f"Opening unrouted benchmark layout: {layout.name}")
    layout.show()
