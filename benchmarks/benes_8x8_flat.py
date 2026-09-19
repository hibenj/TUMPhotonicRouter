"""8x8 Benes with every switch expanded into its primitives.

The netlist the LiDAR bridge hands to the reference router: per switch two
2x2 MMIs, two heater arms and the four internal connections as nets, placed
exactly where `benes_mmi_heater_switch` puts them. Placements and routing
flags are those of `benes_8x8.py`; only the switch cells are flattened
(owner decision 2026-09-19: the comparison with LiDAR needs identical
netlists).
"""

from __future__ import annotations

from gdsfactory.schematic import Schematic

from benchmarks.benes import (
    benes_flat_internal_delays_um,
    benes_flat_node_types,
    benes_flat_topology_metadata,
    build_benes_schematic,
)
from benchmarks.benes_8x8 import NETWORK_SIZE, STABLE_ROUTING_ENV, STABLE_ROUTING_FLAGS

TOPOLOGY_METADATA = benes_flat_topology_metadata(NETWORK_SIZE)
NODE_DEPTHS = TOPOLOGY_METADATA["node_depths"]
NODE_RANKS = TOPOLOGY_METADATA["node_ranks"]
EDGE_RANKS = TOPOLOGY_METADATA["edge_ranks"]
EXPECTED_CROSSINGS = TOPOLOGY_METADATA["crossings"]
NODE_TYPES = benes_flat_node_types(NETWORK_SIZE)
INTERNAL_DELAYS_UM = benes_flat_internal_delays_um(NETWORK_SIZE)

__all__ = [
    "EDGE_RANKS",
    "EXPECTED_CROSSINGS",
    "INTERNAL_DELAYS_UM",
    "NETWORK_SIZE",
    "NODE_DEPTHS",
    "NODE_RANKS",
    "NODE_TYPES",
    "STABLE_ROUTING_ENV",
    "STABLE_ROUTING_FLAGS",
    "TOPOLOGY_METADATA",
    "build_schematic",
]


def build_schematic() -> Schematic:
    """Build the expanded 8x8 Benes benchmark schematic."""
    return build_benes_schematic(NETWORK_SIZE, expand_switches=True)
