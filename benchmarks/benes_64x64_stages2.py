"""Slice of the 64x64 Benes: inputs plus switch stages 0 and 1 only.

Test bed for the first shuffle stage, where the loss-driven 64x64 runs
stalled (2026-09-17): every second net of stage 0 -> 1 needs 1.3-13.7 M
expansions, above the negotiated engine's 2 M / 10 M first budgets. Same
routing flags as `benes_64x64.py`; metadata filtered to the kept nodes and
edges so contribution 1's plan covers exactly the kept crossings.
"""

from __future__ import annotations

from gdsfactory.schematic import Schematic

from benchmarks.benes import build_benes_schematic, input_name, switch_name
from benchmarks.benes_64x64 import (
    EDGE_RANKS as PARENT_EDGE_RANKS,
    EXPECTED_CROSSINGS as PARENT_EXPECTED_CROSSINGS,
    INTERNAL_DELAYS_UM as PARENT_INTERNAL_DELAYS_UM,
    NETWORK_SIZE,
    NODE_DEPTHS as PARENT_NODE_DEPTHS,
    NODE_RANKS as PARENT_NODE_RANKS,
    NODE_TYPES as PARENT_NODE_TYPES,
    STABLE_ROUTING_ENV,
    STABLE_ROUTING_FLAGS,
    STAGE_PITCH_UM,
    TOPOLOGY_METADATA as PARENT_TOPOLOGY_METADATA,
)

STAGE_LIMIT = 2

_KEPT_NODES = {input_name(index) for index in range(NETWORK_SIZE)} | {
    switch_name(stage, index)
    for stage in range(STAGE_LIMIT)
    for index in range(NETWORK_SIZE // 2)
}
_KEPT_EDGES = {
    name
    for name, ranks in PARENT_EDGE_RANKS.items()
    if int(ranks["target_depth"]) <= STAGE_LIMIT
}

NODE_DEPTHS = {name: depth for name, depth in PARENT_NODE_DEPTHS.items() if name in _KEPT_NODES}
NODE_RANKS = {name: rank for name, rank in PARENT_NODE_RANKS.items() if name in _KEPT_NODES}
EDGE_RANKS = {name: ranks for name, ranks in PARENT_EDGE_RANKS.items() if name in _KEPT_EDGES}
EXPECTED_CROSSINGS = tuple(
    crossing
    for crossing in PARENT_EXPECTED_CROSSINGS
    if crossing["edge_a"] in _KEPT_EDGES and crossing["edge_b"] in _KEPT_EDGES
)
NODE_TYPES = {name: kind for name, kind in PARENT_NODE_TYPES.items() if name in _KEPT_NODES}
INTERNAL_DELAYS_UM = {
    name: delay for name, delay in PARENT_INTERNAL_DELAYS_UM.items() if name in _KEPT_NODES
}
TOPOLOGY_METADATA = dict(PARENT_TOPOLOGY_METADATA)

__all__ = [
    "EDGE_RANKS",
    "EXPECTED_CROSSINGS",
    "INTERNAL_DELAYS_UM",
    "NODE_DEPTHS",
    "NODE_RANKS",
    "NODE_TYPES",
    "STABLE_ROUTING_ENV",
    "STABLE_ROUTING_FLAGS",
    "build_schematic",
]


def build_schematic() -> Schematic:
    """Inputs plus switch stages 0 and 1 of the 64x64 Benes."""
    return build_benes_schematic(
        NETWORK_SIZE, stage_pitch_um=STAGE_PITCH_UM, stage_limit=STAGE_LIMIT
    )
