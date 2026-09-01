"""LiDAR multiport MMI 32x32 benchmark."""

from __future__ import annotations

from pathlib import Path

from gdsfactory.schematic import Schematic

from benchmarks.multiportmmi_yaml import build_schematic_from_lidar_yaml

BENCHMARK_YAML = Path(__file__).with_name("data") / "multiportmmi_32x32.yml"

# Inherited from multiportmmi_16x16's stable configuration (owner decision
# 2026-09-01), plus a 4x A* iteration budget: the 32x32 sibling-fanout
# crossings (e.g. net 156's legal 11-crossing thread) need between 5M and
# 20M expansions in their saturated windows -- affordable since the
# eager-completion fix, and cheaper than letting the repair cascade grind
# (its constrained searches are provably empty on this net class). See
# .agent/execplans/2026-09-01-forced-90-degree-route-degradation.md.
STABLE_ROUTING_ENV: dict[str, str] = {
    "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "0.05",
    "PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES": "90",
}

STABLE_ROUTING_FLAGS: tuple[str, ...] = (
    "--crossings",
    "true",
    "--crossing-mode",
    "lidar-pure",
    "--fanout-access-mode",
    "static-stubs",
    "--foreign-port-keepout-cells",
    "0",
    "--max-iterations",
    "20000000",
)

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
