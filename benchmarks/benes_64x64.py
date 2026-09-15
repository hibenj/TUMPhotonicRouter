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
EXPECTED_CROSSINGS = TOPOLOGY_METADATA["crossings"]
NODE_TYPES = benes_node_types(NETWORK_SIZE)
INTERNAL_DELAYS_UM = benes_internal_delays_um(NETWORK_SIZE)


# Stable crossing-router baseline: the same block as benes_32x32 (2026-09-10,
# 64x64 added for the scaling experiment), so that a bare
# `python routing_flow.py benes_64x64` runs the
# lidar-pure configuration the 64x64 baseline is defined on. Without it the
# flow default (`--crossing-mode window`) is used, which is not the baseline.
STABLE_ROUTING_ENV: dict[str, str] = {
    # 1.0 (not the 0.05 of the other benchmarks): experiment E1 of
    # 2026-09-03 -- with the same +-5-cell halo but this weight, stage nets
    # no longer lay long verticals 4 cells next to an existing one (which
    # made the pair uncrossable for every later net); the full 320-net run
    # then passes verification-clean in 15 min. See
    # .agent/execplans/2026-09-03-benes-32x32-lidar-pure-baseline.md.
    "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "1.0",
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
    "--proactive-congestion-weight",
    "4.0",
    "--proactive-congestion-radius-cells",
    "3",
    # Same reasoning as multiportmmi_32x32: stage-2 nets with 10+ mandatory
    # crossings need 5M-20M expansions in their saturated windows; at the
    # 5M default net 59 hits the cap and the repair cascade grinds
    # (see .agent/execplans/2026-09-03-benes-32x32-lidar-pure-baseline.md).
    "--max-iterations",
    "20000000",
)


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
