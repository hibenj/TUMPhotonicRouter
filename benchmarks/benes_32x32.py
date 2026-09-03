"""32x32 Benes network benchmark."""

from __future__ import annotations

from gdsfactory.schematic import Schematic

from benchmarks.benes import (
    benes_internal_delays_um,
    benes_node_types,
    benes_topology_metadata,
    build_benes_schematic,
)

NETWORK_SIZE = 32
TOPOLOGY_METADATA = benes_topology_metadata(NETWORK_SIZE)
NODE_DEPTHS = TOPOLOGY_METADATA["node_depths"]
NODE_RANKS = TOPOLOGY_METADATA["node_ranks"]
EDGE_RANKS = TOPOLOGY_METADATA["edge_ranks"]
EXPECTED_CROSSINGS = TOPOLOGY_METADATA["crossings"]
NODE_TYPES = benes_node_types(NETWORK_SIZE)
INTERNAL_DELAYS_UM = benes_internal_delays_um(NETWORK_SIZE)


# Stable crossing-router baseline (2026-09-03): the same block as
# benes_16x16, so that a bare `python routing_flow.py benes_32x32` runs the
# lidar-pure configuration the 32x32 baseline is defined on. Without it the
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


def build_schematic() -> Schematic:
    """Build the 32x32 Benes benchmark schematic."""
    return build_benes_schematic(NETWORK_SIZE)
