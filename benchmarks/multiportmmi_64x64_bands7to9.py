"""multiportmmi_64x64_bands7to9: slice of multiportmmi_64x64 keeping the columns mmi0_multiport_1 .. mol_array_1
(134 instances, 192 nets), generated 2026-09-15 by
scripts/make_mesh_slice_benchmark.py. Same placements, ports and stable flags
as the parent; net names are the parent's, so the routing order is the
parent's order restricted to these nets."""

from __future__ import annotations

from pathlib import Path

from gdsfactory.schematic import Schematic

from benchmarks.multiportmmi_yaml import build_schematic_from_lidar_yaml
from benchmarks.multiportmmi_64x64 import STABLE_ROUTING_ENV, STABLE_ROUTING_FLAGS  # noqa: F401

BENCHMARK_YAML = Path(__file__).with_name("data") / "multiportmmi_64x64_bands7to9.yml"
N = 64
NODE_TYPES: dict[str, str] = {}
INTERNAL_DELAYS_UM: dict[str, float] = {}


def build_schematic() -> Schematic:
    """Build the multiportmmi_64x64_bands7to9 schematic."""
    return build_schematic_from_lidar_yaml(
        BENCHMARK_YAML, node_types=NODE_TYPES, internal_delays_um=INTERNAL_DELAYS_UM
    )
