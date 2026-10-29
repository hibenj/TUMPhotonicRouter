"""The placement-derived crossing demand equals the depth/rank plan on all 17 benchmarks.

Phase 4 slice C of the 2026-10 do-over replaced the topology analysis
(`topology_analysis.py`, now the test oracle) by `crossing_demand.py` in the
routing flow. For every benchmark the paper routes -- Benes 4x4..128x128,
plain and `_flat`, and the five ADEPT meshes -- the two must agree on the
stages, the wall orders, the pair set, the per-net counts and the ordered
event list per band (level and order index included), because contribution
2 places one tile per event in that order and contribution 1 hands the pairs
to the router. Experiment 5e (2026-10-29) showed the pair sets equal; this
test keeps the whole plan equal. The 128x128 layouts take a few seconds each.
"""

from __future__ import annotations

import pytest

from tests.test_crossing_demand import (
    assert_demand_equals_topology_plan,
    topology_plan_and_demand,
)

PAPER_BENCHMARKS = [
    f"benes_{size}x{size}{suffix}" for size in (4, 8, 16, 32, 64, 128) for suffix in ("", "_flat")
] + [f"multiportmmi_{size}x{size}" for size in (8, 16, 32, 64, 128)]

EXPECTED_BOUNDS = {
    "benes_4x4": 2,
    "benes_8x8": 16,
    "benes_16x16": 88,
    "benes_32x32": 416,
    "benes_64x64": 1824,
    "benes_128x128": 7680,
    "multiportmmi_8x8": 33,
    "multiportmmi_16x16": 63,
    "multiportmmi_32x32": 121,
    "multiportmmi_64x64": 209,
    "multiportmmi_128x128": 311,
}


@pytest.mark.parametrize("name", PAPER_BENCHMARKS)
def test_demand_equals_topology_plan(name: str) -> None:
    plan, demand = topology_plan_and_demand(name)
    assert_demand_equals_topology_plan(plan, demand)
    assert demand.lower_bound() == EXPECTED_BOUNDS[name.removesuffix("_flat")]
