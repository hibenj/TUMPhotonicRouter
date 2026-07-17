"""LiDAR multiport MMI 8x8 benchmark.

This benchmark mirrors the LiDAR ``multiportmmi_8x8`` netlist and placement
YAML so both routers can be run against the same topology.
"""

from __future__ import annotations

from pathlib import Path

from gdsfactory.schematic import Schematic

from benchmarks.multiportmmi_yaml import build_schematic_from_lidar_yaml

BENCHMARK_YAML = Path(__file__).with_name("data") / "multiportmmi_8x8.yml"

N = 8

NODE_TYPES: dict[str, str] = {}
INTERNAL_DELAYS_UM: dict[str, float] = {}

# Stable crossing-router baseline, July 2026:
#   $env:PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05"
#   python routing_flow.py multiportmmi_8x8 --crossings true
#     --crossing-mode lidar-pure --fanout-access-mode static-stubs
#     --routing-window-scale 0.35 --foreign-port-keepout-cells 0
# Static fanout stubs use the router default bend style, currently 90 degrees.
STABLE_ROUTING_ENV: dict[str, str] = {
    "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "0.05",
}

STABLE_ROUTING_FLAGS: tuple[str, ...] = (
    "--crossings",
    "true",
    "--crossing-mode",
    "lidar-pure",
    "--fanout-access-mode",
    "static-stubs",
    "--routing-window-scale",
    "0.35",
    "--foreign-port-keepout-cells",
    "0",
)


def build_schematic() -> Schematic:
    """Build the LiDAR multiport MMI 8x8 benchmark schematic."""
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
