"""`photonic_router.crossing_demand`: the crossing plan from the placement.

The depth/rank analysis (`topology_analysis.py`, the plan's producer until
phase 4 slice C of the 2026-10 do-over) is the oracle here: on the Benes
4x4..32x32 and the ADEPT 8x8 the demand's bands must reproduce its stages,
wall orders, pairs, per-net counts and ordered events exactly. The full 17
benchmark comparison is `tests/e2e/test_crossing_demand_equality.py`.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from collections import Counter
from pathlib import Path

import pytest
from photonic_router.crossing_demand import (
    CHIP_BOUNDARY,
    NON_ADJACENT_WALLS,
    SAME_WALL,
    VERTICAL_PORT,
    Column,
    Terminal,
    _Link,
    build_crossing_demand,
    cluster_columns,
    demand_from_geometry,
)
from photonic_router.crossing_plan import build_crossing_plan
from photonic_router.path_length_graph import PortRef, RoutedEdgeKey
from photonic_router.topology_analysis import analyze_schematic_topology

from benchmark_metadata import load_benchmark_metadata
from translation.layout_from_schematic import layout_from_schematic

REPO_ROOT = Path(__file__).resolve().parents[1]


def topology_plan_and_demand(name: str):
    """(oracle plan, demand) of a benchmark, both from the same schematic and layout."""
    module = importlib.import_module(f"benchmarks.{name}")
    if hasattr(module, "register_benes_cells"):
        module.register_benes_cells()
    schematic = module.build_schematic()
    layout = layout_from_schematic(schematic)
    metadata = load_benchmark_metadata(name, schematic=schematic)
    topology = analyze_schematic_topology(
        schematic,
        node_depths=metadata["node_depths"] or None,
        node_ranks=metadata["node_ranks"] or None,
        edge_ranks=metadata["edge_ranks"] or None,
    )
    return build_crossing_plan(topology.edge_ranks.values()), build_crossing_demand(
        schematic, layout
    )


def _event_record(event) -> tuple:
    return (
        event.edge_a,
        event.edge_b,
        event.source_depth,
        event.target_depth,
        event.level,
        event.order_index,
        event.edge_a_source_rank,
        event.edge_a_target_rank,
        event.edge_b_source_rank,
        event.edge_b_target_rank,
    )


def assert_demand_equals_topology_plan(plan, demand) -> None:
    """Same stages, wall orders, pairs, per-net counts and ordered events."""
    new_plan = demand.plan
    assert set(new_plan.stages) == set(plan.stages)
    assert {(band.index, band.index + 1) for band in demand.bands.values()} == set(plan.stages)
    assert demand.outside == ()

    old_pairs = {event.edge_pair for event in plan.events}
    assert demand.invariant_pairs() == old_pairs
    assert {event.edge_pair for event in new_plan.events} == old_pairs

    old_counts: Counter[str] = Counter()
    for event in plan.events:
        old_counts[event.edge_a.net_name] += 1
        old_counts[event.edge_b.net_name] += 1
    assert demand.counts_by_net_name() == dict(old_counts)
    assert demand.lower_bound() == len(plan.events)

    for key, old_stage in plan.stages.items():
        new_stage = new_plan.stages[key]
        assert new_stage.initial_edge_order == old_stage.initial_edge_order, key
        assert new_stage.final_edge_order == old_stage.final_edge_order, key
        assert [_event_record(e) for e in new_stage.events] == [
            _event_record(e) for e in old_stage.events
        ], key


@pytest.mark.parametrize(
    "name",
    ["benes_4x4", "benes_8x8", "benes_16x16", "benes_32x32", "benes_8x8_flat", "multiportmmi_8x8"],
)
def test_demand_reproduces_the_topology_plan(name: str) -> None:
    plan, demand = topology_plan_and_demand(name)
    assert_demand_equals_topology_plan(plan, demand)


def test_benes_4x4_bands_are_the_stage_transitions() -> None:
    _plan, demand = topology_plan_and_demand("benes_4x4")

    # inputs, three switch stages, outputs: five columns, four bands
    assert [column.index for column in demand.columns] == [0, 1, 2, 3, 4]
    assert sorted(demand.bands) == [0, 1, 2, 3]
    assert demand.columns[0].instances == ("in_0", "in_1", "in_2", "in_3")
    assert {band.index: band.lower_bound for band in demand.bands.values()} == {
        0: 0,
        1: 1,
        2: 1,
        3: 0,
    }
    assert demand.lower_bound() == 2
    band = demand.bands[1]
    assert len(band.nets) == 4
    assert band.x_min == demand.columns[1].x_max
    assert band.x_max == demand.columns[2].x_min
    assert [event.edge_pair for event in band.stage.events] == list(band.invariant_pairs)
    # the first lane on the left wall is the topmost port (y descending)
    assert band.left_order[0].net_name == "n_s0_0_o0_to_s1_0_i0"
    text = demand.to_text()
    assert "CrossingDemand: 2 invariant crossing(s) in 4 band(s) between 5 column(s)" in text
    assert "band 1->2" in text and "left wall:" in text and "right wall:" in text
    assert "0 net(s) outside the band model" in text


def _terminal(instance: str, port: str, x: float, y: float, orientation: float) -> Terminal:
    return Terminal(ref=PortRef(instance, port), x=x, y=y, orientation=orientation)


def _link(net: str, a: Terminal, b: Terminal) -> _Link:
    return _Link(RoutedEdgeKey(net, a.ref, b.ref), a, b)


def test_columns_cluster_overlapping_and_touching_x_extents() -> None:
    boxes = {
        "a": (0.0, 0.0, 10.0, 5.0),
        "b": (5.0, 10.0, 12.0, 15.0),  # overlaps a
        "c": (12.0, 0.0, 20.0, 5.0),  # touches b
        "d": (30.0, 0.0, 40.0, 5.0),
    }
    columns = cluster_columns(boxes)
    assert [column.instances for column in columns] == [("a", "b", "c"), ("d",)]
    assert (columns[0].x_min, columns[0].x_max) == (0.0, 20.0)


def test_nets_outside_the_band_model_are_listed_with_reasons_and_get_no_pairs() -> None:
    columns = (
        Column(0, 0.0, 10.0, ("l0", "l1", "l2")),
        Column(1, 50.0, 60.0, ("r0", "r1")),
        Column(2, 100.0, 110.0, ("f0",)),
    )
    # two confined nets whose terminals alternate: one invariant pair
    swap_a = _link(
        "swap_a", _terminal("l0", "o", 10.0, 20.0, 0.0), _terminal("r0", "i", 50.0, 10.0, 180.0)
    )
    swap_b = _link(
        "swap_b", _terminal("l1", "o", 10.0, 10.0, 0.0), _terminal("r1", "i", 50.0, 20.0, 180.0)
    )
    # declared right to left: still confined, the left wall is its source
    reverse = _link(
        "reverse", _terminal("r1", "i2", 50.0, 0.0, 180.0), _terminal("l2", "o", 10.0, 0.0, 0.0)
    )
    same_wall = _link(
        "same_wall", _terminal("l0", "o2", 10.0, 30.0, 0.0), _terminal("l1", "o2", 10.0, 25.0, 0.0)
    )
    skip = _link(
        "skip", _terminal("l2", "o2", 10.0, -10.0, 0.0), _terminal("f0", "i", 100.0, -10.0, 180.0)
    )
    vertical = _link(
        "vertical", _terminal("l2", "u", 5.0, 5.0, 90.0), _terminal("r0", "i2", 50.0, 5.0, 180.0)
    )
    off_chip = _link(
        "off_chip", _terminal("l0", "w", 0.0, 20.0, 180.0), _terminal("f0", "e", 110.0, 20.0, 0.0)
    )
    demand = demand_from_geometry(
        columns, [swap_a, swap_b, reverse, same_wall, skip, vertical, off_chip]
    )

    assert sorted(demand.bands) == [0]
    band = demand.bands[0]
    assert [edge.net_name for edge in band.left_order] == ["swap_a", "swap_b", "reverse"]
    assert [edge.net_name for edge in band.right_order] == ["swap_b", "swap_a", "reverse"]
    assert band.invariant_pairs == (frozenset((swap_a.edge_key, swap_b.edge_key)),)
    assert demand.lower_bound() == 1
    assert demand.counts_by_net_name() == {"swap_a": 1, "swap_b": 1}
    assert len(band.stage.events) == 1
    assert band.stage.source_depth == 0 and band.stage.target_depth == 1
    assert {net.edge_key.net_name: net.reason for net in demand.outside} == {
        "same_wall": SAME_WALL,
        "skip": NON_ADJACENT_WALLS,
        "vertical": VERTICAL_PORT,
        "off_chip": CHIP_BOUNDARY,
    }
    assert "skips 1 column(s)" in next(
        n.detail for n in demand.outside if n.reason == "non_adjacent_walls"
    )
    assert "outside (same_wall): same_wall" in demand.to_text()


def test_equal_y_terminals_are_ordered_by_x_then_net_name() -> None:
    columns = (Column(0, 0.0, 10.0, ("l",)), Column(1, 50.0, 60.0, ("r",)))
    # three nets leaving one wall at the same y: order by x, then by name
    nets = [
        _link("c", _terminal("l", "o1", 8.0, 0.0, 0.0), _terminal("r", "i1", 50.0, 2.0, 180.0)),
        _link("a", _terminal("l", "o2", 9.0, 0.0, 0.0), _terminal("r", "i2", 50.0, 1.0, 180.0)),
        _link("b", _terminal("l", "o3", 9.0, 0.0, 0.0), _terminal("r", "i3", 50.0, 0.0, 180.0)),
    ]
    demand = demand_from_geometry(columns, nets)
    band = demand.bands[0]
    assert [edge.net_name for edge in band.left_order] == ["c", "a", "b"]
    assert [edge.net_name for edge in band.right_order] == ["c", "a", "b"]
    assert demand.lower_bound() == 0 and band.stage.events == ()


def test_lower_bound_helper_of_the_reproduction_tooling() -> None:
    spec = importlib.util.spec_from_file_location(
        "crossing_lower_bound", REPO_ROOT / "scripts" / "results" / "crossing_lower_bound.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # the script's dataclass resolves its module here
    spec.loader.exec_module(module)

    benes = module.lower_bound_for_benchmark("benes_4x4")
    assert (benes.bound, benes.band_count, benes.outside_count) == (2, 4, 0)
    assert benes.per_net == {
        "n_s0_0_o1_to_s1_1_i0": 1,
        "n_s0_1_o0_to_s1_0_i1": 1,
        "n_s1_0_o1_to_s2_1_i0": 1,
        "n_s1_1_o0_to_s2_0_i1": 1,
    }
    assert benes.gap(2) == 0 and benes.gap(5) == 3 and benes.gap(None) is None
    mesh = module.lower_bound_for_benchmark("multiportmmi_8x8")
    assert mesh.bound == 33 and mesh.outside_count == 0
    assert module.paper_benchmarks()[:2] == ["benes_4x4_flat", "benes_8x8_flat"]
    assert len(module.paper_benchmarks()) == 11
