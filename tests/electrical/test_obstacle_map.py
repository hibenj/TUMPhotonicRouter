"""Tests for translation/electrical/obstacle_extraction.py.

Stage 2 (``ObstacleMapBuilder``): building the layer-filtered electrical
obstacle grid, its role-specific terminal openings and the common-bus rail.
"""

from __future__ import annotations

import pytest

from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from tests.fixtures.synthetic_layouts import (
    single_heater_schematic as build_single_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.net_geometry import _common_bus_tagged_rects
from translation.electrical.obstacle_extraction import build_electrical_obstacle_map
from translation.electrical.pitch_grid import disk_cells
from translation.electrical.terminal_contacts import terminal_contact_seed_points
from translation.electrical.terminal_extraction import extract_heater_terminal_pairs
from translation.layout_from_schematic import layout_from_schematic


def test_obstacle_map_uses_role_specific_terminal_openings():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig()
    groups = extract_heater_terminal_pairs(component, schematic, config)

    obstacle_map = build_electrical_obstacle_map(component, groups, config)
    terminal = groups[0].terminal_a
    all_port_cells: set[tuple[int, int]] = set()
    for point in terminal_contact_seed_points(terminal):
        all_port_cells.update(disk_cells(point, config.terminal_open_radius_um, obstacle_map.grid))
    common_cells = obstacle_map.common_bus_terminal_open_cells[terminal.id]
    individual_cells = obstacle_map.individual_terminal_open_cells[terminal.id]

    assert common_cells
    assert individual_cells
    assert set(common_cells) != all_port_cells
    assert set(individual_cells) != all_port_cells
    assert obstacle_map.terminal_open_cells[terminal.id] == frozenset(
        set(common_cells) | set(individual_cells)
    )


def test_pad_side_derives_opposite_common_bus_side():
    assert ElectricalRoutingConfig(pad_side="top").bus_side == "bottom"
    assert ElectricalRoutingConfig(pad_side="bottom").bus_side == "top"


def test_common_bus_rail_extends_toward_common_bus_pad_side():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        common_bus_pad_position="right",
        bus_x_margin_um=80.0,
        pad_pitch_um=150.0,
        bondpad_width_um=80.0,
    )
    terminal_groups = extract_heater_terminal_pairs(component, schematic, config)
    obstacle_map = build_electrical_obstacle_map(component, terminal_groups, config)

    assert obstacle_map.bus.bbox[2] == pytest.approx(
        obstacle_map.layout_bbox[2]
        + config.bus_x_margin_um
        + config.pad_pitch_um
        + config.common_bus_bondpad_width_um
    )


def test_common_bus_rail_and_pad_escape_use_bus_width_only():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        bus_width_um=60.0,
        wire_width_um=20.0,
        terminal_contact_width_um=10.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    tagged_rects = _common_bus_tagged_rects(
        result.common_bus,
        result.common_bus_escape,
        result.obstacle_map,
        config,
    )
    bus_stripes = [tagged.bbox for tagged in tagged_rects if tagged.source == "bus_stripe"]
    assert bus_stripes
    assert max(ymax - ymin for _, ymin, _, ymax in bus_stripes) == pytest.approx(
        config.bus_width_um
    )

    bus_escape_segments = [tagged.bbox for tagged in tagged_rects if tagged.source == "bus_escape"]
    assert bus_escape_segments
    bus_assignment = result.pad_plan.common_bus_assignment
    assert bus_assignment is not None
    xmin, ymin, xmax, ymax = bus_assignment.slot.bbox
    assert xmax - xmin == pytest.approx(config.common_bus_bondpad_width_um)
    assert ymax - ymin == pytest.approx(config.common_bus_bondpad_length_um)
    assert bus_assignment.slot.side == config.pad_side
    assert result.common_bus_escape is not None
    escape_path = result.common_bus_escape.path
    assert len(escape_path) > 2
    assert escape_path[0] in result.common_bus.bus.cells
    first_y = escape_path[0][1]
    horizontal_prefix = [cell for cell in escape_path if cell[1] == first_y]
    assert len(horizontal_prefix) > 1
    if config.common_bus_pad_position == "right":
        assert horizontal_prefix[-1][0] > horizontal_prefix[0][0]
    else:
        assert horizontal_prefix[-1][0] < horizontal_prefix[0][0]
    bus_connection_cells = {
        cell
        for route in result.common_bus.routes
        for cell in route.path
        if cell in result.common_bus.bus.cells
    }
    assert bus_connection_cells
    origin_x, _ = result.obstacle_map.grid.origin
    grid_pitch = result.obstacle_map.grid.grid_size_um
    connection_xs = [origin_x + (cell[0] + 0.5) * grid_pitch for cell in bus_connection_cells]
    half_overlap_um = max(grid_pitch / 2.0, config.wire_width_um / 2.0)
    assert result.common_bus.bus.bbox[0] == pytest.approx(min(connection_xs) - half_overlap_um)
    assert result.common_bus.bus.bbox[2] == pytest.approx(max(connection_xs) + half_overlap_um)
    bus_route_segments = [tagged.bbox for tagged in tagged_rects if tagged.source == "bus_route"]
    assert bus_route_segments
    for xmin, ymin, xmax, ymax in bus_route_segments:
        assert min(xmax - xmin, ymax - ymin) <= config.wire_width_um
    if result.common_bus.bus_side == "bottom":
        assert all(ymin >= result.common_bus.bus.bbox[1] for _, ymin, _, _ in bus_route_segments)
    else:
        assert all(ymax <= result.common_bus.bus.bbox[3] for _, _, _, ymax in bus_route_segments)
