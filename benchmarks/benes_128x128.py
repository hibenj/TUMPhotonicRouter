"""128x128 Benes network benchmark."""

from __future__ import annotations

from gdsfactory.schematic import Schematic

from benchmarks.benes import (
    benes_internal_delays_um,
    benes_node_types,
    benes_topology_metadata,
    build_benes_schematic,
)

NETWORK_SIZE = 128
TOPOLOGY_METADATA = benes_topology_metadata(NETWORK_SIZE)
NODE_DEPTHS = TOPOLOGY_METADATA["node_depths"]
NODE_RANKS = TOPOLOGY_METADATA["node_ranks"]
EDGE_RANKS = TOPOLOGY_METADATA["edge_ranks"]
NODE_TYPES = benes_node_types(NETWORK_SIZE)
INTERNAL_DELAYS_UM = benes_internal_delays_um(NETWORK_SIZE)


# Stage pitch 2500 um (2026-09-13): the outer shuffle layers of the 128x128
# have 63 swap levels (the 64x64 has 31 at 1500 um, the 32x32 has 15 at
# 1000 um). Same rule as the 64x64: the same room per swap level as the
# 32x32 has at 1000 um (29.6 um); 63 levels need 63 * 29.6 + 556 um of
# component width and margins = 2421 um, rounded up to 2500 um (30.9 um per
# level). Only contribution 2 is run at this size (owner decision
# 2026-09-13); the loss-driven configurations time out already at 64x64
# and are recorded as failed.
STAGE_PITCH_UM = 2500.0


def build_schematic() -> Schematic:
    """Build the 128x128 Benes benchmark schematic."""
    return build_benes_schematic(NETWORK_SIZE, stage_pitch_um=STAGE_PITCH_UM)
