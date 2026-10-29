"""Focused tests for the pre-placed crossing tile derivation.

These exercise the tile placement and the schematic/layout derivation for the
four Benes benchmarks without routing anything: crossing counts against the
placement's crossing demand, lane movement analysis, tile snapping, and -- most importantly
-- that no two waveguide pieces inside a placed tile physically overlap (a
collision that end-to-end verification cannot see, because tile geometry is a
placed instance, not a routed record).
"""

from __future__ import annotations

import importlib
from dataclasses import replace

import klayout.db as kdb
import pytest
from gdsfactory.gpdk import get_generic_pdk
from photonic_router.crossing_plan import CrossingStagePlan

from translation import preplaced_crossing_grids as pcg
from translation.layout_from_schematic import layout_from_schematic

get_generic_pdk().activate()

BENES_EXPECTED_CROSSINGS = {"benes_4x4": 2, "benes_8x8": 16, "benes_16x16": 88, "benes_32x32": 416}


def _benchmark(name: str):
    module = importlib.import_module(f"benchmarks.{name}")
    schematic = module.build_schematic()
    schedule = pcg.build_stage_schedule_for_benchmark(schematic, layout_from_schematic(schematic))
    return schematic, schedule


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
def test_derived_tiles_cover_every_planned_crossing_without_internal_collisions(name: str) -> None:
    schematic, schedule = _benchmark(name)
    assert schedule.event_count == BENES_EXPECTED_CROSSINGS[name]
    derived = pcg.derive_preplaced_crossing_layout(
        schematic, layout_from_schematic(schematic), schedule
    )
    assert derived.placed_crossing_count == BENES_EXPECTED_CROSSINGS[name]
    assert derived.expected_crossing_count == BENES_EXPECTED_CROSSINGS[name]
    assert derived.plan_event_count == BENES_EXPECTED_CROSSINGS[name]
    for instance_name, build in derived.grid_builds.items():
        assert _internal_overlap_count(build.component) == 0, instance_name


@pytest.mark.parametrize("name", sorted(BENES_EXPECTED_CROSSINGS))
def test_derived_schematic_splits_only_crossing_lanes(name: str) -> None:
    schematic, schedule = _benchmark(name)
    derived = pcg.derive_preplaced_crossing_layout(
        schematic, layout_from_schematic(schematic), schedule
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
    # Every added net is a segment of a split lane (one ``__via<i>`` net per
    # extra crossing of a lane, plus the ``__to_grid``/``__from_grid`` ends).
    added = derived_nets - original_nets
    assert len(added) >= 2 * len(split)
    for net_name in added:
        base = net_name.split("__")[0]
        assert base in split, net_name
    for instance_name in derived.grid_instance_names:
        assert instance_name in derived.unrouted_layout.insts
    # Every tile must fit in the free band between switch columns (~535 um).
    for build in derived.grid_builds.values():
        assert build.width_um < 500.0


def test_lane_movement_rejects_direction_reversal() -> None:
    _schematic, schedule = _benchmark("benes_4x4")
    stage_plan = schedule.stages[0].stage
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


def test_x_array_tiles_snap_to_the_routing_grid_so_port_cells_are_footprint_edge_cells():
    """benes_64x64 (2026-09-11): a tile whose centre sat mid-cell pushed its
    footprint one cell past the port cell, and the router's diagonal halo
    rule rejected the last 45-degree step into the port. With the routing
    configuration known, every X-array tile centre lies on a grid cell
    boundary in x, and both port cells on each axis are the footprint's edge
    cells."""
    import importlib
    import math

    from photonic_router.static_obstacle_builder import StaticObstacleMapConfig

    from translation.layout_from_schematic import layout_from_schematic
    from translation.preplaced_crossing_grids import (
        CROSSING_TILE_PREFIX,
        _port_cells_stay_inside_footprint,
        build_stage_schedule_for_benchmark,
        derive_preplaced_crossing_layout,
    )

    probe = {
        "fanout_access_mode": "static-stubs",
        "bend_radius_um": 5.0,
        "obstacle_config": StaticObstacleMapConfig(grid_size_um=2.0),
    }
    module = importlib.import_module("benchmarks.benes_8x8")
    schematic = module.build_schematic()
    layout = layout_from_schematic(schematic)
    schedule = build_stage_schedule_for_benchmark(schematic, layout)
    derived = derive_preplaced_crossing_layout(
        schematic, layout, schedule, router_probe_kwargs=probe
    )
    grid = 2.0
    origin_x = float(layout.dbbox().left) - 20.0
    origin_y = float(layout.dbbox().bottom) - 20.0
    tiles = [
        r
        for r in derived.unrouted_layout.insts
        if str(r.name).startswith(f"{CROSSING_TILE_PREFIX}stage")
    ]
    assert len(tiles) == 16
    for ref in tiles:
        cx, cy = ref.dcenter
        cells = (cx - origin_x) / grid
        assert math.isclose(cells, round(cells), abs_tol=1e-6), (ref.name, cx)
        assert _port_cells_stay_inside_footprint(cx, origin_x, grid), (ref.name, "x")
        assert _port_cells_stay_inside_footprint(cy, origin_y, grid), (ref.name, "y")


# ---------------------------------------------------------------------------
# phase 4 slice D: the stage schedule and the band-layered refusal
# ---------------------------------------------------------------------------


def _lane_port_band(schematic, layout, stage_plan) -> tuple[float, float]:
    """The free band the tiles builder measures: max source port x to min target port x."""
    from translation.route_gds import get_port_from_instance

    source_xs, target_xs = [], []
    for edge in stage_plan.initial_edge_order:
        p1, p2 = pcg._net_endpoints(schematic, edge.net_name)
        source_xs.append(float(get_port_from_instance(layout, *p1.split(",")).dcenter[0]))
        target_xs.append(float(get_port_from_instance(layout, *p2.split(",")).dcenter[0]))
    return max(source_xs), min(target_xs)


@pytest.mark.parametrize(
    ("name", "stage_count", "right_recess_um"),
    [("benes_8x8", 4, 0.0), ("multiportmmi_8x8", 5, 15.9)],
)
def test_the_tiles_free_band_starts_at_the_schedule_bands_left_wall_and_ends_at_the_ports(
    name: str, stage_count: int, right_recess_um: float
) -> None:
    """The tiles builder keeps measuring its free band between the lanes' own
    port faces; slice D did not switch it to the schedule's wall extent
    because the two differ: on the Benes cells they are equal, on the
    multiport-MMI cells the target ports of three stages sit 15.9 um inside
    the right wall column (its face is set by a longer instance of the same
    column), so the port band reaches that far past the wall face and the
    column grid's columns are centred on it. Switching would move them."""
    schematic, schedule = _benchmark(name)
    layout = layout_from_schematic(schematic)
    assert len(schedule) == stage_count
    assert schedule.keys == tuple(sorted(schedule.keys))
    recesses = []
    for band in schedule:
        assert band.key == (band.stage.source_depth, band.stage.target_depth)
        assert band.nets == band.stage.initial_edge_order
        port_x0, port_x1 = _lane_port_band(schematic, layout, band.stage)
        assert port_x0 == pytest.approx(band.x_min, abs=1e-6)
        assert port_x1 >= band.x_max - 1e-6
        recesses.append(round(port_x1 - band.x_max, 3))
    assert set(recesses) <= {0.0, right_recess_um}
    if right_recess_um:
        assert recesses.count(right_recess_um) == 3


def test_the_derivation_reports_one_layer_decision_per_scheduled_stage() -> None:
    schematic, schedule = _benchmark("benes_8x8")
    derived = pcg.derive_preplaced_crossing_layout(
        schematic, layout_from_schematic(schematic), schedule
    )
    assert tuple(d.stage_key for d in derived.layer_decisions) == schedule.keys
    assert derived.plan_event_count == schedule.event_count == 16
    # tile names carry the stage's source column index (`source_depth`)
    for band in schedule:
        prefix = f"{pcg.CROSSING_TILE_PREFIX}stage{band.index}_"
        assert any(n.startswith(prefix) for n in derived.grid_instance_names), prefix


def _non_layered_benes_4x4():
    """benes_4x4 with two nets the band model does not cover."""
    from gdsfactory.schematic import Net

    schematic = importlib.import_module("benchmarks.benes_4x4").build_schematic()
    # both terminals on the right wall of the stage-0 switches
    schematic.add_net(Net(p1="sw_s0_0,o3", p2="sw_s0_1,o3", name="probe_same_wall"))
    # a stage-0 output straight to the output column, skipping stage 1
    schematic.add_net(Net(p1="sw_s0_0,o4", p2="out_0,o1", name="probe_skip"))
    return schematic


def test_build_stage_schedule_refuses_a_non_layered_design_with_nets_and_reasons() -> None:
    schematic = _non_layered_benes_4x4()
    with pytest.raises(ValueError) as info:
        pcg.build_stage_schedule_for_benchmark(schematic, layout_from_schematic(schematic))
    assert str(info.value) == (
        "2 net(s) outside the band model (non_adjacent_walls: probe_skip; "
        "same_wall: probe_same_wall)"
    )


def test_the_flow_stage_refuses_a_non_layered_design_before_placing_anything() -> None:
    from photonic_router.config import RoutingConfig
    from photonic_router.flow import _preplaced_crossing_grids_stage

    schematic = _non_layered_benes_4x4()
    with pytest.raises(RuntimeError) as info:
        _preplaced_crossing_grids_stage(
            schematic=schematic,
            unrouted_layout=layout_from_schematic(schematic),
            total_steps=5,
            stats=None,
            config=RoutingConfig(),
        )
    assert str(info.value) == (
        "pre-placed crossing grids need a band-layered design: 2 net(s) outside the "
        "band model (non_adjacent_walls: probe_skip; same_wall: probe_same_wall); "
        "route with --crossing-mode lidar-pure or lidar-guided"
    )
    assert isinstance(info.value.__cause__, ValueError)
