"""16x16 Benes network benchmark."""

from __future__ import annotations

from gdsfactory.schematic import Schematic

from benchmarks.benes import (
    benes_internal_delays_um,
    benes_node_types,
    benes_topology_metadata,
    build_benes_schematic,
)

NETWORK_SIZE = 16
TOPOLOGY_METADATA = benes_topology_metadata(NETWORK_SIZE)
NODE_DEPTHS = TOPOLOGY_METADATA["node_depths"]
NODE_RANKS = TOPOLOGY_METADATA["node_ranks"]
EDGE_RANKS = TOPOLOGY_METADATA["edge_ranks"]
EXPECTED_CROSSINGS = TOPOLOGY_METADATA["crossings"]
NODE_TYPES = benes_node_types(NETWORK_SIZE)
INTERNAL_DELAYS_UM = benes_internal_delays_um(NETWORK_SIZE)

# Stable crossing-router baseline, July 2026:
#   $env:PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05"
#   python routing_flow.py benes_16x16 --crossings true --crossing-mode lidar-pure
#     --fanout-access-mode static-stubs --foreign-port-keepout-cells 0
# Static fanout stubs use the router default bend style, currently 90 degrees.
STABLE_ROUTING_ENV: dict[str, str] = {
    "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT": "0.05",
    # Pinned to the old Weighted-A* behavior: at the repository default of
    # 1.0 (admissible A*) one net of this benchmark hits the known
    # "crossing-aware endpoint correction produced no realizable centerline"
    # hole (n_s0_6_o0_to_s1_3_i0). Remove this pin once that hole is fixed --
    # see .agent/execplans/2026-08-31-admissible-astar-heuristic-45-degree.md.
    "PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT": "1.25",
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
)


def build_schematic() -> Schematic:
    """Build the 16x16 Benes benchmark schematic."""
    return build_benes_schematic(NETWORK_SIZE)
