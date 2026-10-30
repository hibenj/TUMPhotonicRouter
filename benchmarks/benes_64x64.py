"""64x64 Benes network benchmark."""

from __future__ import annotations

from gdsfactory.schematic import Schematic

from benchmarks.benes import (
    benes_internal_delays_um,
    benes_node_types,
    benes_topology_metadata,
    build_benes_schematic,
)

NETWORK_SIZE = 64
TOPOLOGY_METADATA = benes_topology_metadata(NETWORK_SIZE)
NODE_DEPTHS = TOPOLOGY_METADATA["node_depths"]
NODE_RANKS = TOPOLOGY_METADATA["node_ranks"]
EDGE_RANKS = TOPOLOGY_METADATA["edge_ranks"]
NODE_TYPES = benes_node_types(NETWORK_SIZE)
INTERNAL_DELAYS_UM = benes_internal_delays_um(NETWORK_SIZE)


# Stage pitch 1500 um instead of the generator's 1000 um (2026-09-10): the
# outer shuffle layers of the 64x64 have 31 swap levels (the 32x32 has 15).
# The rule is the same room per swap level as the 32x32 has at 1000 um
# (29.6 um): 31 levels need a 1500 um pitch (30.5 um per level). At 1000 um
# a level gets 14.3 um and no crossing structure fits; at 1100 um (17.5 um)
# the tiles fit but the router cannot connect consecutive levels whose lanes
# run far vertically (two 45-degree bends do not fit into the gap). The
# placement is an input of every configuration, so all four runs of the
# scaling plan use this pitch.
STAGE_PITCH_UM = 1500.0


def build_schematic() -> Schematic:
    """Build the 64x64 Benes benchmark schematic."""
    return build_benes_schematic(NETWORK_SIZE, stage_pitch_um=STAGE_PITCH_UM)
