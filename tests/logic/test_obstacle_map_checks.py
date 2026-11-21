"""`photonic_router.logic_checks.obstacle_map`: each statement finds what it names.

The map is a real one (`heater_single`, built as a session without a debug
directory builds it); each test breaks one thing in a copy.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from photonic_router.logic_checks import ERROR, count_by_check
from photonic_router.logic_checks.obstacle_map import check_obstacle_map
from photonic_router.records.layout_from_schematic import layout_from_schematic
from photonic_router.records.route_rust_obstacle_config import (
    resolve_obstacle_config,
    with_bbox_cell_materialization,
)
from photonic_router.static_obstacle_builder import build_static_obstacle_map

from benchmarks.heater_single import build_schematic

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def design():
    schematic = build_schematic()
    config = with_bbox_cell_materialization(
        resolve_obstacle_config(None, route_layer=(1, 0), include_heater_obstacles=False),
        materialize_bbox_cells=False,
        populate_obstacle_map=False,
    )
    return schematic, build_static_obstacle_map(layout_from_schematic(schematic), config), config


def _errors(schematic, obstacle_map, config) -> dict[str, int]:
    findings = check_obstacle_map(schematic, obstacle_map, config)
    return count_by_check(finding for finding in findings if finding.severity == ERROR)


def test_a_map_of_one_layer_group_has_no_error(design) -> None:
    assert _errors(*design) == {}


def test_two_layer_groups_count_every_port_once() -> None:
    """With heater obstacles the map is merged from two groups that both see all ports."""
    schematic = build_schematic()
    config = resolve_obstacle_config(None, route_layer=(1, 0), include_heater_obstacles=True)
    obstacle_map = build_static_obstacle_map(layout_from_schematic(schematic), config)
    assert obstacle_map.build_stats["port_count"] == len(obstacle_map.benchmark.ports)
    assert obstacle_map.build_stats["port_open_cell_count"] == len(obstacle_map.port_open_cells)
    findings = check_obstacle_map(schematic, obstacle_map, config)
    # the one disagreement left: blocked_cell_count adds up the groups' box areas
    assert [f.subject for f in findings if f.severity == ERROR] == ["blocked_cell_count"]


def test_the_blocked_area_is_the_cells_and_the_rectangles(design) -> None:
    schematic, obstacle_map, config = design
    assert len(obstacle_map.blocked_static_rects) > 0
    without_rects = replace(obstacle_map, blocked_static_rects=())
    errors = _errors(schematic, without_rects, config)
    assert errors["obstacle_map.obstacle_not_blocked"] == len(obstacle_map.benchmark.polygons)


def test_cells_and_rectangles_lie_on_the_grid(design) -> None:
    schematic, obstacle_map, config = design
    grid = obstacle_map.grid
    broken = replace(
        obstacle_map,
        port_open_cells={*obstacle_map.port_open_cells, (grid.width, 0)},
        chip_boundary_rects=(*obstacle_map.chip_boundary_rects, (0, -1, 3, 3)),
    )
    assert _errors(schematic, broken, config)["obstacle_map.cell_outside_grid"] == 2


def test_the_keep_out_band_stays_outside_the_routable_box(design) -> None:
    schematic, obstacle_map, config = design
    grid = obstacle_map.grid
    middle = (grid.width // 2, grid.height // 2)
    broken = replace(
        obstacle_map, chip_boundary_rects=(*obstacle_map.chip_boundary_rects, (*middle, *middle))
    )
    findings = check_obstacle_map(schematic, broken, config)
    (inside,) = [f for f in findings if f.check == "obstacle_map.keep_out_inside_routable_box"]
    assert inside.detail == "1 cell(s)"


def test_the_cell_in_front_of_a_linked_port_is_free_or_opened(design) -> None:
    schematic, obstacle_map, config = design
    grid = obstacle_map.grid
    everything = ((0, 0, grid.width - 1, grid.height - 1),)
    walled_in = replace(obstacle_map, blocked_static_rects=everything, port_open_cells=set())
    links = sum(len(bundle.links) for bundle in schematic.netlist.routes.values())
    assert _errors(schematic, walled_in, config)["obstacle_map.port_exit_blocked"] == 2 * links


def test_a_linked_port_is_among_the_extracted_ports(design) -> None:
    schematic, obstacle_map, config = design
    no_ports = replace(obstacle_map, benchmark=replace(obstacle_map.benchmark, ports=[]))
    links = sum(len(bundle.links) for bundle in schematic.netlist.routes.values())
    errors = _errors(schematic, no_ports, config)
    assert errors["obstacle_map.linked_port_not_extracted"] == 2 * links


def test_the_boxes_nest_grid_die_design(design) -> None:
    schematic, obstacle_map, config = design
    left, bottom, right, top = obstacle_map.grid.die_bbox
    wide_die = replace(obstacle_map.grid, die_bbox=(left - 100.0, bottom, right, top))
    assert _errors(schematic, replace(obstacle_map, grid=wide_die), config) == {
        "obstacle_map.grid_misses_die": 1
    }
    far = replace(obstacle_map, routable_bbox=(left - 100.0, bottom, right, top))
    assert "obstacle_map.routable_box_outside_die" in _errors(schematic, far, config)


def test_the_build_stats_count_what_the_map_holds(design) -> None:
    schematic, obstacle_map, config = design
    stats = dict(obstacle_map.build_stats)
    stats["port_count"] *= 2
    findings = check_obstacle_map(schematic, replace(obstacle_map, build_stats=stats), config)
    (finding,) = [f for f in findings if f.severity == ERROR]
    assert (finding.check, finding.subject) == ("obstacle_map.stats_disagree", "port_count")
