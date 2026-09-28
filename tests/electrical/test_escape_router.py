"""Tests for translation/electrical/escape_router.py.

Stage 8 (``BusEscapeRouter``): routing the common bus tree to its assigned
abstract pad slot.
"""

from __future__ import annotations

import pytest

from tests.fixtures.synthetic_layouts import (
    single_heater_schematic as build_single_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.net_geometry import _common_bus_tagged_rects
from translation.layout_from_schematic import layout_from_schematic


def test_common_bus_escape_reaches_assigned_common_bus_pad_slot():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.pad_plan is not None
    assert result.common_bus_escape is not None
    escape = result.common_bus_escape
    assert escape.success, escape.reason
    assert escape.pad_assignment == result.pad_plan.common_bus_assignment
    assert escape.path[0] in result.common_bus.tree_cells
    assert escape.path[-1] in escape.target_cells
    assert escape.target_cells
    assert escape.cost == len(escape.path) - 1
    pad_bbox = escape.pad_assignment.slot.bbox
    bus_escape_rects = [
        tagged.bbox
        for tagged in _common_bus_tagged_rects(
            result.common_bus,
            result.common_bus_escape,
            result.obstacle_map,
            config,
        )
        if tagged.source == "bus_escape"
    ]
    assert any(
        rect[0] == pytest.approx(pad_bbox[0]) and rect[2] == pytest.approx(pad_bbox[2])
        for rect in bus_escape_rects
    )


def test_common_bus_escape_uses_opposite_bus_for_bottom_pad_side():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="bottom",
        pad_pitch_um=150.0,
        pad_origin_x_um=0.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.common_bus.bus_side == "top"
    assert result.pad_plan is not None
    assert result.pad_plan.side == "bottom"
    assert result.common_bus_escape is not None
    assert result.common_bus_escape.success, result.common_bus_escape.reason
    assert result.common_bus_escape.path[0] in result.common_bus.tree_cells
    assert result.common_bus_escape.path[-1] in result.common_bus_escape.target_cells
