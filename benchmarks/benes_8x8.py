"""8x8 Benes network benchmark."""

from __future__ import annotations

from gdsfactory.schematic import Schematic

from benchmarks.benes import (
    benes_internal_delays_um,
    benes_node_types,
    benes_topology_metadata,
    build_benes_schematic,
)

NETWORK_SIZE = 8
TOPOLOGY_METADATA = benes_topology_metadata(NETWORK_SIZE)
NODE_DEPTHS = TOPOLOGY_METADATA["node_depths"]
NODE_RANKS = TOPOLOGY_METADATA["node_ranks"]
EDGE_RANKS = TOPOLOGY_METADATA["edge_ranks"]
EXPECTED_CROSSINGS = TOPOLOGY_METADATA["crossings"]
NODE_TYPES = benes_node_types(NETWORK_SIZE)
INTERNAL_DELAYS_UM = benes_internal_delays_um(NETWORK_SIZE)


def build_schematic() -> Schematic:
    """Build the 8x8 Benes benchmark schematic."""
    return build_benes_schematic(NETWORK_SIZE)


if __name__ == "__main__":
    from translation.layout_from_schematic import layout_from_schematic

    schematic = build_schematic()
    print("Schematic instances:", list(schematic.netlist.instances.keys()))
    print("Schematic placements:", list(schematic.placements.keys()))
    print("\nNets defined:")
    for net_name, bundle in schematic.netlist.routes.items():
        print(f"  - {net_name}: {bundle.links}")
    print(f"\nTotal nets: {len(schematic.netlist.routes)}")

    layout = layout_from_schematic(schematic)
    print(f"Opening unrouted benchmark layout: {layout.name}")
    layout.show()
