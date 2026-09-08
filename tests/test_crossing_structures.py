"""The crossing-structure selector (contribution 2): per layer, every
alignment reports feasibility with a reason and a cost; the cheapest feasible
one is chosen; a layer nothing fits is a router fallback."""

from __future__ import annotations

import importlib

import pytest
from gdsfactory.gpdk import get_generic_pdk
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig

from benchmark_metadata import load_benchmark_metadata
from translation import crossing_structures as cs
from translation import preplaced_crossing_grids as pcg
from translation.layout_from_schematic import layout_from_schematic

get_generic_pdk().activate()

PROBE_KWARGS = {
    "fanout_access_mode": "static-stubs",
    "bend_radius_um": 5.0,
    "obstacle_config": StaticObstacleMapConfig(grid_size_um=2.0),
}


def _decisions(name: str) -> list[cs.LayerDecision]:
    module = importlib.import_module(f"benchmarks.{name}")
    if hasattr(module, "register_benes_cells"):
        module.register_benes_cells()
    schematic = module.build_schematic()
    layout = layout_from_schematic(schematic)
    plan = pcg.build_crossing_plan_for_benchmark(
        schematic, load_benchmark_metadata(name, schematic=schematic)
    )
    # the probe must see the routing run's configuration (2 um grid, static
    # stubs, bend radius 5), or the anchors differ from the run's
    derived = pcg.derive_preplaced_crossing_layout(
        schematic, layout, plan, router_probe_kwargs=PROBE_KWARGS
    )
    return [d for d in derived.layer_decisions if isinstance(d, cs.LayerDecision)]


def test_benes_layers_take_the_x_array_and_report_the_column_grid_cost() -> None:
    decisions = _decisions("benes_8x8")
    assert len(decisions) == 4
    for decision in decisions:
        assert decision.chosen == cs.X_ARRAY
        x = next(c for c in decision.candidates if c.name == cs.X_ARRAY)
        assert x.feasible and x.cost is not None and x.cost.extra_length_um == 0.0
        # the column grid is evaluated too: either feasible with a positive
        # extra length or infeasible with a stated reason
        column = next(c for c in decision.candidates if c.name == cs.COLUMN_GRID)
        assert (column.feasible and column.cost.extra_length_um > 0.0) or column.reason


def test_multiportmmi_layers_take_the_column_grid_with_a_reason_against_the_x_array() -> None:
    decisions = _decisions("multiportmmi_8x8")
    assert len(decisions) == 5
    for decision in decisions:
        assert decision.chosen == cs.COLUMN_GRID
        x = next(c for c in decision.candidates if c.name == cs.X_ARRAY)
        assert not x.feasible
        assert "opposite" in x.reason or "dense" in x.reason or "reverses" in x.reason
        column = next(c for c in decision.candidates if c.name == cs.COLUMN_GRID)
        assert column.feasible and column.cost is not None and column.cost.corners > 0


def test_a_layer_nothing_fits_is_a_router_fallback_with_reasons(monkeypatch) -> None:
    # single-port lanes at 50 um pitch with a random permutation: the X array
    # needs opposite directions, the column grid finds cyclic constraints
    monkeypatch.setenv("PHOTONIC_ROUTER_SYNTH", "k=1,m=8,sp=0,tp=50,band=300,mode=random,seed=1")
    import benchmarks.permutation_synthetic as synth

    importlib.reload(synth)
    schematic = synth.build_schematic()
    layout = layout_from_schematic(schematic)
    plan = pcg.build_crossing_plan_for_benchmark(
        schematic, load_benchmark_metadata("permutation_synthetic", schematic=schematic)
    )
    stage = next(s for s in plan.stages.values() if s.events)
    decision = cs.select_layer_structure(
        schematic, layout, stage, pcg.CrossingGridGeometry(), {}, stage_key=(0, 1)
    )
    assert decision.chosen == cs.ROUTER
    assert all(not c.feasible and c.reason for c in decision.candidates)
    # default: the layer's nets are left to the guided router, unsplit
    derived = pcg.derive_preplaced_crossing_layout(schematic, layout, plan)
    assert derived.router_fallback_net_names == {e.net_name for e in stage.initial_edge_order}
    assert derived.placed_crossing_count == 0
    assert all(
        name in derived.schematic.netlist.routes for name in derived.router_fallback_net_names
    )
    with pytest.raises(ValueError, match="no pre-placed crossing structure fits"):
        pcg.derive_preplaced_crossing_layout(
            schematic, layout, plan, geometry=pcg.CrossingGridGeometry(router_fallback=False)
        )
