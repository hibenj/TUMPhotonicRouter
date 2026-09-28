"""Tests for translation/electrical/terminal_contacts.py and port_access.py.

Terminal contact selection, terminal access-path trimming, and the
role-specific port access anchors built off them.
"""

from __future__ import annotations

from types import SimpleNamespace

from translation.electrical.port_access import build_terminal_port_access
from translation.electrical.terminal_contacts import select_terminal_contact, terminal_access_path
from translation.electrical.types import ElectricalPortRef, ElectricalTerminal


def test_terminal_contact_selects_physical_port_not_logical_center():
    terminal = ElectricalTerminal(
        id="heater_0:l",
        heater_id="heater_0",
        side_key="l",
        center=(50.0, 50.0),
        bbox=(38.0, 38.0, 62.0, 62.0),
        ports=(
            ElectricalPortRef("l_e1", (40.0, 50.0), 180.0, 4.0, (49, 0)),
            ElectricalPortRef("l_e2", (50.0, 60.0), 90.0, 4.0, (49, 0)),
            ElectricalPortRef("l_e3", (60.0, 50.0), 0.0, 4.0, (49, 0)),
            ElectricalPortRef("l_e4", (50.0, 40.0), 270.0, 4.0, (49, 0)),
        ),
        layer=(49, 0),
    )

    contact_center, contact_bbox = select_terminal_contact(
        terminal,
        route_start_um=(0.0, 50.0),
        fallback_width_um=20.0,
    )

    assert contact_center == (40.0, 50.0)
    assert contact_center != terminal.center
    assert contact_bbox == (30.0, 40.0, 50.0, 60.0)


def test_terminal_access_path_trims_snapped_points_inside_terminal():
    terminal = ElectricalTerminal(
        id="heater_0:l",
        heater_id="heater_0",
        side_key="l",
        center=(50.0, 50.0),
        bbox=(38.0, 38.0, 62.0, 62.0),
        ports=(ElectricalPortRef("l_e2", (50.0, 60.0), 90.0, 4.0, (49, 0)),),
        layer=(49, 0),
    )

    access = terminal_access_path(
        terminal,
        route_points_um=((50.0, 60.0), (50.0, 65.0), (50.0, 90.0)),
        fallback_width_um=10.0,
    )

    assert access.contact_center == (50.0, 60.0)
    assert access.access_width_um == 10.0
    # The tail keeps the real boundary-crossing segment (keepout edge -> the
    # snapped point) instead of collapsing the whole run into the adapter, so
    # a route_tail rectangle -- not an oversized terminal_adapter one --
    # carries the wire outside the terminal's keepout.
    assert access.adapter_points == ((50.0, 60.0), (50.0, 67.0))
    assert access.route_tail_points == ((50.0, 67.0), (50.0, 90.0))


def test_terminal_access_path_can_pin_selected_physical_port():
    terminal = ElectricalTerminal(
        id="heater_0:l",
        heater_id="heater_0",
        side_key="l",
        center=(50.0, 50.0),
        bbox=(38.0, 38.0, 62.0, 62.0),
        ports=(
            ElectricalPortRef("l_e1", (40.0, 50.0), 180.0, 4.0, (49, 0)),
            ElectricalPortRef("l_e3", (60.0, 50.0), 0.0, 4.0, (49, 0)),
        ),
        layer=(49, 0),
    )

    access = terminal_access_path(
        terminal,
        route_points_um=((100.0, 50.0),),
        fallback_width_um=10.0,
        preferred_port_name="l_e1",
    )

    assert access.contact_center == (40.0, 50.0)
    assert access.contact_bbox == (35.0, 45.0, 45.0, 55.0)


def test_build_terminal_port_access_skips_blocked_candidate_cell():
    grid = SimpleNamespace(width=8, height=8, grid_size_um=10.0, origin=(0.0, 0.0))
    terminal = ElectricalTerminal(
        id="heater_0:t",
        heater_id="heater_0",
        side_key="t",
        center=(23.0, 24.0),
        bbox=(18.0, 18.0, 28.0, 28.0),
        ports=(ElectricalPortRef("top", (23.0, 29.0), 90.0, 4.0, (49, 0)),),
        layer=(49, 0),
    )

    access = build_terminal_port_access(
        terminal,
        purpose="individual",
        side="top",
        opened_cells=frozenset({(2, 2), (2, 3)}),
        blocked_cells=frozenset({(2, 3)}),
        grid=grid,
        fallback_width_um=10.0,
    )

    assert access.anchor_cell == (2, 2)
    assert access.port_point_um == (23.0, 29.0)
    assert access.access_centerline_um[0] == access.port_point_um
    assert access.access_centerline_um[-1] == access.anchor_point_um


def test_build_terminal_port_access_uses_side_specific_port():
    grid = SimpleNamespace(width=8, height=8, grid_size_um=10.0, origin=(0.0, 0.0))
    terminal = ElectricalTerminal(
        id="heater_0:x",
        heater_id="heater_0",
        side_key="x",
        center=(40.0, 40.0),
        bbox=(35.0, 35.0, 45.0, 45.0),
        ports=(
            ElectricalPortRef("bottom", (40.0, 35.0), 270.0, 4.0, (49, 0)),
            ElectricalPortRef("top", (40.0, 45.0), 90.0, 4.0, (49, 0)),
        ),
        layer=(49, 0),
    )

    top_access = build_terminal_port_access(
        terminal,
        purpose="individual",
        side="top",
        opened_cells=frozenset({(4, 4)}),
        blocked_cells=frozenset(),
        grid=grid,
        fallback_width_um=10.0,
    )
    bottom_access = build_terminal_port_access(
        terminal,
        purpose="common_bus",
        side="bottom",
        opened_cells=frozenset({(4, 3)}),
        blocked_cells=frozenset(),
        grid=grid,
        fallback_width_um=10.0,
    )

    assert top_access.port_name == "top"
    assert bottom_access.port_name == "bottom"
