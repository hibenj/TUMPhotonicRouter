"""Helpers for LiDAR multiport MMI YAML benchmarks."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Placement, Schematic


def load_lidar_yaml(path: Path) -> dict[str, Any]:
    """Load a LiDAR benchmark YAML file.

    Current upstream LiDAR YAML files can contain ``!!python/tuple`` tags in
    component settings, so ``FullLoader`` is required. The files are benchmark
    data checked into this repository, not user-supplied input.
    """
    if not path.exists():
        raise FileNotFoundError(f"Benchmark YAML not found: {path}")
    data = yaml.load(path.read_text(encoding="utf-8-sig"), Loader=yaml.FullLoader)
    if not isinstance(data, dict):
        raise TypeError(f"Expected mapping in benchmark YAML: {path}")
    return data


def clean_instance_settings(settings: dict[str, Any]) -> dict[str, Any]:
    cleaned = dict(settings)
    cleaned.pop("macro_type", None)
    cleaned.pop("placement", None)
    cleaned.pop("info", None)
    return cleaned


def placement_from_lidar(raw: dict[str, Any]) -> Placement:
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


def classify_node(instance_name: str, component: str) -> str:
    if instance_name == "gc1":
        return "input"
    if instance_name.startswith("gc_array_out_"):
        return "output"
    if component in {"mmi", "mmi1x2", "mzi"}:
        return "gate"
    return "delay"


def build_schematic_from_lidar_yaml(
    path: Path,
    *,
    node_types: dict[str, str],
    internal_delays_um: dict[str, float],
) -> Schematic:
    """Build a gdsfactory schematic from a LiDAR benchmark YAML file."""
    get_generic_pdk().activate()

    data = load_lidar_yaml(path)
    schematic = Schematic()

    loaded_node_types: dict[str, str] = {}
    loaded_internal_delays: dict[str, float] = {}

    placements = data.get("schematic_placements", {})
    if not isinstance(placements, dict):
        raise TypeError(f"Expected schematic_placements mapping in {path}")

    instances = data.get("instances", {})
    if not isinstance(instances, dict):
        raise TypeError(f"Expected instances mapping in {path}")

    for instance_name, raw_instance in instances.items():
        if not isinstance(raw_instance, dict):
            raise TypeError(f"Expected instance mapping for {instance_name!r}")
        component = str(raw_instance["component"])
        raw_settings = raw_instance.get("settings", {})
        if not isinstance(raw_settings, dict):
            raise TypeError(f"Expected settings mapping for {instance_name!r}")
        settings = clean_instance_settings(dict(raw_settings))
        placement_data = placements.get(instance_name)
        if placement_data is None:
            raise ValueError(f"No placement found for LiDAR instance {instance_name!r}")
        if not isinstance(placement_data, dict):
            raise TypeError(f"Expected placement mapping for {instance_name!r}")

        schematic.add_instance(
            str(instance_name),
            Instance(component=component, settings=settings),
            placement_from_lidar(dict(placement_data)),
        )
        loaded_node_types[str(instance_name)] = classify_node(str(instance_name), component)
        loaded_internal_delays[str(instance_name)] = 0.0

    nets = data.get("nets", {})
    if not isinstance(nets, dict):
        raise TypeError(f"Expected nets mapping in {path}")
    for net_name, endpoints in nets.items():
        if len(endpoints) != 2:
            raise ValueError(f"Expected two endpoints for net {net_name!r}, got {endpoints!r}")
        schematic.add_net(Net(p1=str(endpoints[0]), p2=str(endpoints[1]), name=str(net_name)))

    node_types.clear()
    node_types.update(loaded_node_types)
    internal_delays_um.clear()
    internal_delays_um.update(loaded_internal_delays)

    return schematic
