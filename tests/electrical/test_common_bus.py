"""Tests for translation/electrical/common_bus_router.py, column_trunks.py and
pad_side_reconciliation.py.

Stages 3 (``CommonBusRouter``), 4 (``CommonBusTrimmer``) and 7
(``PadSideReconciler``): connecting one terminal per heater to the common
bus, its column-trunk sharing, and the pad-side consistency swap.
"""

from __future__ import annotations

from dataclasses import replace

from tests.electrical.support import (
    _column_trunk_vertical_cells,
    _dbu_bbox,
    _kdb_box,
    _kdb_region,
    _path_direction_changes,
    _terminal,
    _verification_obstacle_map,
)
from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.bus_search import terminal_open_cells as _terminal_open_cells
from translation.electrical.common_bus_router import route_common_bus
from translation.electrical.net_geometry import _common_bus_tagged_rects
from translation.electrical.types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    TerminalBusRoute,
)
from translation.electrical.verification import verify_electrical_routing
from translation.layout_from_schematic import layout_from_schematic


def test_common_bus_router_selects_exactly_one_terminal_per_heater(tmp_path):
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(
        component,
        schematic,
        config,
        debug_dir=tmp_path,
        debug_prefix="mmi8x4",
    )

    group_ids = {group.heater_id for group in result.terminal_groups}
    assert len(group_ids) == 20
    assert result.common_bus.success
    assert set(result.common_bus.selected_terminals) == group_ids
    assert set(result.common_bus.unselected_terminals) == group_ids
    for group in result.terminal_groups:
        selected = result.common_bus.selected_terminals[group.heater_id]
        unselected = result.common_bus.unselected_terminals[group.heater_id]
        assert selected.id != unselected.id
        assert {selected.id, unselected.id} == {group.terminal_a.id, group.terminal_b.id}
    # 2026-09-28 (Milestone 3, column trunks): a route may legitimately cross
    # another terminal's open cells when that terminal is also selected for
    # the common bus (same net; the column trunk lands a contact at each
    # member it passes). Only terminals *not* selected for the bus stay
    # forbidden ground.
    selected_terminal_ids = {
        terminal.id for terminal in result.common_bus.selected_terminals.values()
    }
    for route in result.common_bus.routes:
        other_terminal_cells = set().union(
            *(
                cells
                for terminal_id, cells in result.obstacle_map.common_bus_terminal_open_cells.items()
                if terminal_id not in selected_terminal_ids
            )
        )
        assert not set(route.path).intersection(other_terminal_cells)

    svg_path = tmp_path / "electrical" / "mmi8x4_common_bus.svg"
    assert svg_path.exists()
    svg = svg_path.read_text(encoding="utf-8")
    assert svg.startswith("<svg")
    assert "<polyline" in svg
    assert "detailed bundle=" in svg
    assert "topology bundle=" not in svg
    assert "individual route pad=" not in svg
    metal_snapshot_path = tmp_path / "electrical" / "mmi8x4_metal_snapshot.svg"
    assert result.debug_artifacts["metal_snapshot_svg"] == str(metal_snapshot_path)
    assert metal_snapshot_path.exists()
    metal_snapshot = metal_snapshot_path.read_text(encoding="utf-8")
    assert metal_snapshot.startswith("<svg")
    assert "electrical metal snapshot" in metal_snapshot
    assert "realized metal" in metal_snapshot
    assert "terminal contact" in metal_snapshot


def test_common_bus_terminal_selection_prefers_local_same_row_pair_midpoint():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        common_bus_terminal_selection="local_pair_median_x_biased",
    )

    result = route_electrical_heaters(component, schematic, config)

    groups_by_id = {group.heater_id: group for group in result.terminal_groups}
    expected_pairs = (
        ("heater_0", "heater_post_0"),
        ("heater_1", "heater_post_1"),
        ("heater_2", "heater_post_2"),
        ("heater_3", "heater_post_3"),
        ("heater_output_0", "heater_final_0"),
        ("heater_output_1", "heater_final_1"),
        ("heater_output_2", "heater_final_2"),
        ("heater_output_3", "heater_final_3"),
        # heater_extra_1/heater_extra_2 excluded: Milestone 4's pad-side
        # consistency swap (route_electrical._apply_pad_side_consistency_swap)
        # legitimately overrides the pure midpoint choice for heater_extra_2,
        # since it is not part of a bus column group and its individual
        # terminal's exit pointed away from its pad.
    )
    for left_id, right_id in expected_pairs:
        left_group = groups_by_id[left_id]
        right_group = groups_by_id[right_id]
        left_center_x = (left_group.terminal_a.center[0] + left_group.terminal_b.center[0]) / 2.0
        right_center_x = (right_group.terminal_a.center[0] + right_group.terminal_b.center[0]) / 2.0
        midpoint_x = (left_center_x + right_center_x) / 2.0
        for group in (left_group, right_group):
            selected = result.common_bus.selected_terminals[group.heater_id]
            unselected = result.common_bus.unselected_terminals[group.heater_id]
            assert abs(selected.center[0] - midpoint_x) <= abs(unselected.center[0] - midpoint_x)


def test_common_bus_column_trunks_serve_stacked_terminals_with_one_stub_bend_each():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    routes_by_id = {route.heater_id: route for route in result.common_bus.routes}
    for column_heater_ids in (
        ("heater_0", "heater_1", "heater_2", "heater_3"),
        ("heater_post_0", "heater_post_1", "heater_post_2", "heater_post_3"),
    ):
        column_routes = [routes_by_id[heater_id] for heater_id in column_heater_ids]
        exit_dx = (
            1 if result.common_bus.selected_terminals[column_heater_ids[0]].side_key == "r" else -1
        )
        trunk_xs: set[int] = set()
        union_cells: set[tuple[int, int]] = set()
        for route in column_routes:
            assert _path_direction_changes(route.path) <= 1
            assert route.route_start_cell == route.path[0]
            assert route.access_anchor_cell is not None
            assert route.path[0] in _terminal_open_cells(result.obstacle_map, route.terminal.id)
            assert exit_dx * (route.path[0][0] - route.access_anchor_cell[0]) >= 0
            vertical_cells = _column_trunk_vertical_cells(route.path)
            trunk_xs.add(route.path[-1][0])
            union_cells.update(vertical_cells)

        assert len(trunk_xs) == 1
        trunk_x = next(iter(trunk_xs))
        ys = sorted(cell[1] for cell in union_cells)
        assert ys == list(range(ys[0], ys[-1] + 1))
        extreme_cells = {(trunk_x, ys[0]), (trunk_x, ys[-1])}
        assert extreme_cells & result.obstacle_map.bus.cells

    assert result.common_bus.selected_terminals["heater_0"].side_key == "r"
    assert result.common_bus.selected_terminals["heater_post_0"].side_key == "l"


def test_common_bus_column_trunk_falls_back_below_an_obstacle():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig()

    result = route_electrical_heaters(component, schematic, config)

    obstacle_map = result.obstacle_map
    routes_by_id = {route.heater_id: route for route in result.common_bus.routes}
    column_routes = [
        routes_by_id[heater_id] for heater_id in ("heater_0", "heater_1", "heater_2", "heater_3")
    ]
    trunk_xs = {route.path[-1][0] for route in column_routes}
    assert len(trunk_xs) == 1
    trunk_x = next(iter(trunk_xs))

    heater_1_anchor = routes_by_id["heater_1"].access_anchor_cell
    heater_2_anchor = routes_by_id["heater_2"].access_anchor_cell
    assert heater_1_anchor is not None and heater_2_anchor is not None
    y0, y1 = sorted((heater_1_anchor[1], heater_2_anchor[1]))
    assert y1 - y0 >= 2
    blocked_cell = (trunk_x, (y0 + y1) // 2)
    assert y0 < blocked_cell[1] < y1

    patched_map = replace(
        obstacle_map,
        blocked_cells=frozenset(obstacle_map.blocked_cells | {blocked_cell}),
    )

    patched_result = route_common_bus(result.terminal_groups, patched_map, config)

    assert patched_result.failed_heaters == ()
    patched_routes_by_id = {route.heater_id: route for route in patched_result.routes}
    served_trunk_xs = set()
    for heater_id in ("heater_2", "heater_3"):
        route = patched_routes_by_id[heater_id]
        served_trunk_xs.add(route.path[-1][0])
        assert _path_direction_changes(route.path) <= 1
    assert len(served_trunk_xs) == 1
    served_trunk_x = next(iter(served_trunk_xs))
    for heater_id in ("heater_0", "heater_1"):
        route = patched_routes_by_id[heater_id]
        assert not all(cell[0] == served_trunk_x for cell in route.path)
    for route in patched_result.routes:
        assert blocked_cell not in route.path


def test_branch_and_escape_sharing_a_bus_column_have_no_redundant_overlap():
    """A bus branch and the bus escape leaving from the same column must meet at
    exactly one junction: the branch's intentional penetration into the stripe,
    with the escape's own rectangles starting flush at the stripe boundary
    instead of padding backward past it. Regression for the geometry slip where
    the escape's un-clipped leading end-cap reproduced part of the stripe's own
    footprint, and a branch's small stripe entry fell inside that reproduced
    area, counted as a same-net redundant overlap between "bus_route" and
    "bus_escape".
    """

    obstacle_map = _verification_obstacle_map()
    config = ElectricalRoutingConfig(
        pad_side="top",
        routing_grid_pitch_um=10.0,
        wire_width_um=4.0,
        bus_width_um=10.0,
        terminal_contact_width_um=4.0,
    )
    # The branch lands on the bus at column x=3 -- the same column the escape
    # leaves from (cell (3, 0), the bus's rightmost cell).
    terminal = _terminal("heater_0:l", (35.0, 205.0))
    branch_path = tuple((3, y) for y in range(20, -1, -1))
    common_bus = CommonBusRoutingResult(
        bus_side="bottom",
        bus=obstacle_map.bus,
        selected_terminals={"heater_0": terminal},
        unselected_terminals={},
        routes=(
            TerminalBusRoute(
                heater_id="heater_0",
                terminal=terminal,
                path=branch_path,
                cost=len(branch_path) - 1,
            ),
        ),
        tree_cells=frozenset(obstacle_map.bus.cells),
    )
    # The escape leaves the bus at the branch's own column, runs along the row
    # for long enough that the leading end-cap trim is not a no-op, then turns
    # up toward its pad.
    escape = CommonBusEscapeResult(
        pad_assignment=None,
        path=((3, 0), (4, 0), (5, 0), (6, 0), (6, 1)),
        target_cells=frozenset({(6, 1)}),
        success=True,
    )

    verification = verify_electrical_routing(
        obstacle_map,
        common_bus,
        common_bus_escape=escape,
        detailed_bundle_routes=None,
        pad_plan=None,
        config=config,
    )

    assert verification.metrics["same_net_redundant_overlap_pair_count"] == 0
    assert verification.metrics["metal_redundant_area_overcount_um2"] == 0

    tagged_rects = _common_bus_tagged_rects(common_bus, escape, obstacle_map, config)
    route_rects = [tagged.bbox for tagged in tagged_rects if tagged.source == "bus_route"]
    escape_rects = [tagged.bbox for tagged in tagged_rects if tagged.source == "bus_escape"]
    stripe_rects = [tagged.bbox for tagged in tagged_rects if tagged.source == "bus_stripe"]
    assert route_rects and escape_rects and stripe_rects

    # The escape must never cover part of the stripe's own footprint.
    for escape_rect in escape_rects:
        for stripe_rect in stripe_rects:
            overlap = (
                max(escape_rect[0], stripe_rect[0]),
                max(escape_rect[1], stripe_rect[1]),
                min(escape_rect[2], stripe_rect[2]),
                min(escape_rect[3], stripe_rect[3]),
            )
            assert overlap[2] <= overlap[0] or overlap[3] <= overlap[1]

    # The junction is still one connected polygon: branch, stripe, and escape
    # touch with no gap.
    region = _kdb_region()
    for bbox in (*route_rects, *escape_rects, *stripe_rects):
        region.insert(_kdb_box(*_dbu_bbox(bbox)))
    merged = tuple(region.merged().each())
    assert len(merged) == 1


def test_pad_side_reconciliation_swaps_a_lone_heater_and_drops_its_branch_straight(
    electrical_benchmark_result,
):
    """``heater_s_mod``'s ``heater_extra_0`` is the fixture already known (fb15e4a)
    to force ``pad_side_reconciliation.reconcile_pad_sides``'s swap: its
    original individual terminal's exit pointed away from its pad, so its
    bus/pad-wire roles are swapped and its bus branch is re-planned with a
    straight drop (``column_trunks.straight_drop_to_bus``) instead of a tree
    holding its own discarded branch.
    """

    result = electrical_benchmark_result("heater_s_mod")

    assert result.common_bus.selected_terminals["heater_extra_0"].side_key == "l"

    route = next(route for route in result.common_bus.routes if route.heater_id == "heater_extra_0")
    assert len({cell[0] for cell in route.path}) == 1
    assert route.path[-1] in result.common_bus.bus.cells
