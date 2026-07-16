"""LiDAR multiport MMI 32x32 benchmark."""

from __future__ import annotations

from pathlib import Path

from gdsfactory.schematic import Schematic

from benchmarks.multiportmmi_yaml import build_schematic_from_lidar_yaml

BENCHMARK_YAML = Path(__file__).with_name("data") / "multiportmmi_32x32.yml"

N = 32

NODE_TYPES: dict[str, str] = {}
INTERNAL_DELAYS_UM: dict[str, float] = {}


def build_schematic() -> Schematic:
    """Build the LiDAR multiport MMI 32x32 benchmark schematic."""
    return build_schematic_from_lidar_yaml(
        BENCHMARK_YAML,
        node_types=NODE_TYPES,
        internal_delays_um=INTERNAL_DELAYS_UM,
    )


if __name__ == "__main__":
    from translation.layout_from_schematic import layout_from_schematic

    schematic = build_schematic()
    print("Schematic instances:", len(schematic.netlist.instances))
    print("Schematic placements:", len(schematic.placements))
    print("Total nets:", len(schematic.netlist.routes))

    layout = layout_from_schematic(schematic)
    print(f"Opening unrouted benchmark layout: {layout.name}")
    layout.show()
