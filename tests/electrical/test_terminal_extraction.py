"""Tests for translation/electrical/terminal_extraction.py.

Stage 1 (``TerminalExtractor``): extracting each heater's two interchangeable
electrical terminals from a routed layout.
"""

from __future__ import annotations

from tests.fixtures.synthetic_layouts import (
    single_heater_schematic as build_single_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig
from translation.electrical.terminal_extraction import extract_heater_terminal_pairs
from translation.layout_from_schematic import layout_from_schematic


def test_extracts_two_logical_terminals_from_multi_port_heater():
    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)

    groups = extract_heater_terminal_pairs(component, schematic, ElectricalRoutingConfig())

    assert len(groups) == 1
    group = groups[0]
    assert group.heater_id == "heater_0"
    assert {group.terminal_a.side_key, group.terminal_b.side_key} == {"l", "r"}
    assert len(group.terminal_a.ports) == 4
    assert len(group.terminal_b.ports) == 4
    assert group.terminal_a.center[0] < group.terminal_b.center[0]
