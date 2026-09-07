"""Focused tests for the pre-placed crossing grid generator and derivation.

These exercise the grid geometry and the schematic/layout derivation for the
three Benes benchmarks without routing anything: port counts and ordering,
crossing counts against the topology, lattice movement analysis, fan column
ordering, and -- most importantly -- that no two waveguide pieces inside a grid
physically overlap (a collision that end-to-end verification cannot see,
because grid geometry is a placed instance, not a routed record).
"""

from __future__ import annotations

import importlib
from dataclasses import replace

import klayout.db as kdb
import pytest
from gdsfactory.gpdk import get_generic_pdk

from benchmark_metadata import load_benchmark_metadata
from photonic_router.crossing_plan import CrossingEvent, CrossingStagePlan
from translation import preplaced_crossing_grids as pcg
from translation.layout_from_schematic import layout_from_schematic

get_generic_pdk().activate()

BENES_EXPECTED_CROSSINGS = {"benes_4x4": 2, "benes_8x8": 16, "benes_16x16": 88, "benes_32x32": 416}


def _benchmark(name: str):
    module = importlib.import_module(f"benchmarks.{name}")
    schematic = module.build_schematic()
    metadata = load_benchmark_metadata(name, schematic=schematic)
    plan = pcg.build_crossing_plan_for_benchmark(schematic, metadata)
    return schematic, plan


def _piece_regions(component) -> list[kdb.Region]:
    layer = component.kcl.layer(1, 0)
    regions = []
    for inst in component.insts:
        region = kdb.Region(inst.cell.begin_shapes_rec(layer))
        region.transform(inst.cplx_trans)
        region.merge()
        regions.append(region)
    return regions


def _internal_overlap_count(component, *, min_area_um2: float = 0.05) -> int:
    regions = _piece_regions(component)
    dbu = component.kcl.dbu
    count = 0
    for index, left in enumerate(regions):
        for right in regions[index + 1 :]:
            overlap = left & right
            if overlap.is_empty():
                continue
            if overlap.area() * dbu * dbu > min_area_um2:
                count += 1
    return count


@pytest.mark.parametrize("name", sorted(BENES_EXPECTED_CROSSINGS))
def test_participating_runs_cover_every_crossing(name: str) -> None:
    _schematic, plan = _benchmark(name)
    total = 0
    for stage_plan in plan.stages.values():
        runs = pcg.split_stage_into_participating_runs(stage_plan)
        run_events = sum(len(run.events) for run in runs)
        assert run_events == len(stage_plan.events)
        total += run_events
        for run in runs:
            participating = {e.edge_a for e in run.events} | {e.edge_b for e in run.events}
            assert set(run.initial_edge_order) == participating
            assert set(run.final_edge_order) == participating
            levels = sorted({event.level for event in run.events})
            assert levels == list(range(len(levels)))
    assert total == BENES_EXPECTED_CROSSINGS[name]


@pytest.mark.parametrize("name", sorted(BENES_EXPECTED_CROSSINGS))
def test_grid_ports_and_crossings_match_the_stage_plan(name: str) -> None:
    """Compact lattice (fan_mode "router"); the stretched mode's ports are
    covered by test_stretched_grid_puts_every_crossing_at_the_midpoint..."""
    _schematic, plan = _benchmark(name)
    for stage_plan in plan.stages.values():
        for run in pcg.split_stage_into_participating_runs(stage_plan):
            build = pcg.build_crossing_grid_component(
                run, pcg.CrossingGridGeometry(fan_mode="router")
            )
            component = build.component
            lane_count = len(run.initial_edge_order)
            assert build.crossing_count == len(run.events)
            in_ports = [component.ports[f"in_{i}"] for i in range(lane_count)]
            out_ports = [component.ports[f"out_{i}"] for i in range(lane_count)]
            assert all(port.orientation == 180 for port in in_ports)
            assert all(port.orientation == 0 for port in out_ports)
            # Top to bottom: strictly decreasing y on both faces.
            in_ys = [float(port.dcenter[1]) for port in in_ports]
            out_ys = [float(port.dcenter[1]) for port in out_ports]
            assert in_ys == sorted(in_ys, reverse=True)
            assert out_ys == sorted(out_ys, reverse=True)
            for index, edge in enumerate(run.initial_edge_order):
                assert build.input_port_name_by_net_name[edge.net_name] == f"in_{index}"
            for index, edge in enumerate(run.final_edge_order):
                assert build.output_port_name_by_net_name[edge.net_name] == f"out_{index}"
            assert all(length > 0.0 for length in build.lane_length_um_by_net_name.values())


@pytest.mark.parametrize("fan_mode", ["router", "stretched"])
@pytest.mark.parametrize("name", sorted(BENES_EXPECTED_CROSSINGS))
def test_derived_grids_have_no_internal_waveguide_collisions(name: str, fan_mode: str) -> None:
    schematic, plan = _benchmark(name)
    geometry = pcg.CrossingGridGeometry(fan_mode=fan_mode)
    derived = pcg.derive_preplaced_crossing_layout(
        schematic, layout_from_schematic(schematic), plan, geometry=geometry
    )
    assert derived.placed_crossing_count == BENES_EXPECTED_CROSSINGS[name]
    assert derived.expected_crossing_count == BENES_EXPECTED_CROSSINGS[name]
    for instance_name, build in derived.grid_builds.items():
        assert _internal_overlap_count(build.component) == 0, instance_name


def _crossing_centers(component) -> list[tuple[float, float]]:
    centers = []
    for inst in component.insts:
        if "crossing" in inst.cell.name:
            box = inst.dbbox()
            centers.append((round(float(box.center().x), 3), round(float(box.center().y), 3)))
    return centers


def test_stretched_grid_puts_every_crossing_at_the_midpoint_of_the_rows_it_swaps() -> None:
    """Owner rule (2026-09-07): a crossing sits at the average position of the
    two lanes it swaps -- the midpoint of their slot rows -- and the swap
    levels are columns spread evenly over the band."""
    _schematic, plan = _benchmark("benes_8x8")
    stage_plan = next(p for p in plan.stages.values() if len(p.events) == 6)
    run = pcg.split_stage_into_participating_runs(stage_plan)[0]
    rows = {
        edge.net_name: 300.0 - 110.0 * i - (11.0 if i % 2 else -11.0)
        for i, edge in enumerate(run.initial_edge_order)
    }
    geometry = pcg.CrossingGridGeometry(fan_mode="stretched")
    build = pcg.build_crossing_grid_component(
        run, geometry, entry_row_by_net=rows, band_width_um=544.5
    )
    centers = _crossing_centers(build.component)
    assert len(centers) == 6
    usable = 544.5 - 2 * geometry.band_margin_um
    pitch = usable / 3
    slot_rows = [rows[edge.net_name] for edge in run.initial_edge_order]
    _movement, crossings = pcg._lane_movement(run)
    expected = set()
    for level, upper_slot, _upper, _lower in crossings:
        x = -usable / 2 + (level + 0.5) * pitch
        y = 0.5 * (slot_rows[upper_slot] + slot_rows[upper_slot + 1])
        expected.add((round(x, 3), round(y, 3)))
    assert set(centers) == expected
    assert build.width_um == pytest.approx(usable)
    assert _internal_overlap_count(build.component) == 0
    # every lane enters on its own row and leaves on its final slot row
    for index in range(len(run.initial_edge_order)):
        port = build.component.ports[f"in_{index}"]
        assert float(port.dcenter[1]) == pytest.approx(slot_rows[index])
    for index in range(len(run.final_edge_order)):
        port = build.component.ports[f"out_{index}"]
        assert float(port.dcenter[1]) == pytest.approx(slot_rows[index])


def test_stretched_grid_rejects_a_band_too_narrow_for_its_levels() -> None:
    _schematic, plan = _benchmark("benes_8x8")
    stage_plan = next(p for p in plan.stages.values() if len(p.events) == 6)
    run = pcg.split_stage_into_participating_runs(stage_plan)[0]
    rows = {edge.net_name: 300.0 - 110.0 * i for i, edge in enumerate(run.initial_edge_order)}
    with pytest.raises(ValueError, match="per level"):
        pcg.build_crossing_grid_component(
            run,
            pcg.CrossingGridGeometry(fan_mode="stretched"),
            entry_row_by_net=rows,
            band_width_um=100.0,
        )


@pytest.mark.parametrize("name", sorted(BENES_EXPECTED_CROSSINGS))
def test_derived_schematic_splits_only_crossing_lanes(name: str) -> None:
    schematic, plan = _benchmark(name)
    derived = pcg.derive_preplaced_crossing_layout(
        schematic, layout_from_schematic(schematic), plan
    )
    original_nets = set(schematic.netlist.routes)
    derived_nets = set(derived.schematic.netlist.routes)
    split = set(derived.stub_nets_by_original_net)
    assert split <= original_nets
    for net_name in original_nets - split:
        assert net_name in derived_nets
    for net_name, (to_name, from_name) in derived.stub_nets_by_original_net.items():
        assert net_name not in derived_nets
        assert to_name in derived_nets and from_name in derived_nets
        assert derived.grid_internal_length_um_by_original_net[net_name] > 0.0
    # Every added net is a segment of a split lane (tiles mode adds one
    # ``__via<i>`` net per extra crossing of a lane; the lattice modes add
    # exactly two nets per lane).
    added = derived_nets - original_nets
    assert len(added) >= 2 * len(split)
    for net_name in added:
        base = net_name.split("__")[0]
        assert base in split, net_name
    for instance_name in derived.grid_instance_names:
        assert instance_name in derived.unrouted_layout.insts
    # Every grid must fit in the free band between switch columns (~535 um).
    for build in derived.grid_builds.values():
        assert build.width_um < 500.0


def test_lane_movement_rejects_direction_reversal() -> None:
    _schematic, plan = _benchmark("benes_4x4")
    stage_plan = next(plan for plan in plan.stages.values() if plan.events)
    event = stage_plan.events[0]
    # Swap the same pair back at the next level: a reversal the lattice cannot draw.
    reversed_plan = CrossingStagePlan(
        source_depth=stage_plan.source_depth,
        target_depth=stage_plan.target_depth,
        initial_edge_order=stage_plan.initial_edge_order,
        final_edge_order=stage_plan.initial_edge_order,
        events=(event, replace(event, level=event.level + 1, order_index=event.order_index + 1)),
    )
    with pytest.raises(ValueError, match="reverses direction"):
        pcg._lane_movement(reversed_plan)


def test_fan_column_order_follows_rows_not_travel() -> None:
    # Lane "outer" is farther from the core but travels less than "inner".
    far_rows = {"outer": 109.0, "inner": 106.0}
    slot_rows = {"outer": 35.0, "inner": 15.0}
    entry = pcg._fan_column_order(far_rows, slot_rows, closest_first=True)
    assert entry == {"inner": 0, "outer": 1}
    exit_order = pcg._fan_column_order(far_rows, slot_rows, closest_first=False)
    assert exit_order == {"outer": 0, "inner": 1}
    # Lanes on opposite sides of the core share column indices.
    both = pcg._fan_column_order(
        {"up": 100.0, "down": -100.0}, {"up": 10.0, "down": -10.0}, closest_first=True
    )
    assert both == {"up": 0, "down": 0}


def test_sibling_ports_are_spread_symmetrically() -> None:
    rows = {"a": 0.625, "b": -0.625, "c": 220.0}
    instances = {"a": "sw", "b": "sw", "c": "other"}
    spread = pcg._spread_sibling_rows(rows, instances, 2.0)
    assert spread["a"] == pytest.approx(2.0)
    assert spread["b"] == pytest.approx(-2.0)
    assert spread["c"] == pytest.approx(220.0)


def test_grid_geometry_rejects_pitch_too_small_for_bends() -> None:
    _schematic, plan = _benchmark("benes_4x4")
    stage_plan = next(plan for plan in plan.stages.values() if plan.events)
    run = pcg.split_stage_into_participating_runs(stage_plan)[0]
    with pytest.raises(ValueError):
        pcg.build_crossing_grid_component(run, pcg.CrossingGridGeometry(lane_pitch_um=6.0))
