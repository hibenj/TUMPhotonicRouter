"""Tests for translation/electrical/common_bus_router.py, column_trunks.py and
pad_side_reconciliation.py.

Stages 3 (``CommonBusRouter``), 4 (``CommonBusTrimmer``) and 7
(``PadSideReconciler``): connecting one terminal per heater to the common
bus, its column-trunk sharing, and the pad-side consistency swap.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

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
from tests.fixtures.synthetic_layouts import (
    single_heater_schematic as build_single_heater_schematic,
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
        obstacle_clearance_um=10.0,
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
        obstacle_clearance_um=10.0,
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
        obstacle_clearance_um=10.0,
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


# Also marked e2e (routes a full 12-mode cascade benchmark end to end,
# ~6 s): keeps the fast "-m not e2e" suite under its enforced 60 s budget
# (scripts/test_baseline.sh).
@pytest.mark.integration
@pytest.mark.e2e
def test_swap_skips_a_heater_whose_branch_carries_others(electrical_benchmark_result):
    """Rule C (eight-row-bundles plan): a heater is skipped by the pad-side
    consistency swap when another heater's bus branch has joined it (its
    path's tree-connection cell, ``path[-1]``, lies on the candidate's own
    path) -- swapping would discard that branch and strand the one that
    joined it (the ``common_bus_route_disconnected`` bug the eight-row-
    bundles plan's Fact 3 recorded on ``cascade_16_diagonal``'s
    ``heater_extra_3``/``heater_extra_4``).

    Since the straight-drop phase of 2026-10-05 (compact-diagonal plan) no
    benchmark produces such a greedy join any more -- every lone cascade
    heater drops straight to the stripe -- so the rule is exercised on the
    real ``cascade_12_diagonal`` objects with two edits: ``heater_extra_4``'s
    individual pad is moved to the far side of its terminal (a pad-side
    mismatch, so the heater is a swap candidate: without a carry the swap
    happens), and ``heater_extra_3``'s branch is made to join
    ``heater_extra_4``'s path (with the carry the swap is refused).
    """

    from dataclasses import replace

    from translation.electrical.pad_side_reconciliation import (
        _apply_pad_side_consistency_swap,
    )

    result = electrical_benchmark_result("cascade_12_diagonal")
    config = ElectricalRoutingConfig()
    common_bus = result.common_bus
    obstacle_map = result.obstacle_map
    heater_id = "heater_extra_4"
    individual = common_bus.unselected_terminals[heater_id]
    exit_dx = 1 if individual.side_key == "r" else -1

    assignments = []
    for assignment in result.pad_plan.assignments:
        if assignment.kind == "individual" and assignment.terminal.id == individual.id:
            slot = assignment.slot
            far_x = individual.center[0] - exit_dx * 500.0
            half_width = (slot.bbox[2] - slot.bbox[0]) / 2.0
            slot = replace(
                slot,
                center=(far_x, slot.center[1]),
                bbox=(far_x - half_width, slot.bbox[1], far_x + half_width, slot.bbox[3]),
            )
            assignment = replace(assignment, slot=slot)
        assignments.append(assignment)
    mismatched_plan = replace(result.pad_plan, assignments=tuple(assignments))

    # without a carry: the mismatch is real and the heater is swapped
    swapped = _apply_pad_side_consistency_swap(obstacle_map, common_bus, mismatched_plan, config)
    assert swapped is not None and heater_id in swapped[1]

    # with a carry: heater_extra_3's branch joins heater_extra_4's path, so
    # heater_extra_4 keeps its bus terminal
    routes_by_heater_id = {route.heater_id: route for route in common_bus.routes}
    route_4 = routes_by_heater_id[heater_id]
    route_3 = routes_by_heater_id["heater_extra_3"]
    join_cell = route_4.path[len(route_4.path) // 2]
    carried = replace(route_3, path=(*route_3.path[:-1], join_cell))
    carrying_bus = replace(
        common_bus,
        routes=tuple(
            carried if route.heater_id == "heater_extra_3" else route for route in common_bus.routes
        ),
    )
    swapped = _apply_pad_side_consistency_swap(obstacle_map, carrying_bus, mismatched_plan, config)
    assert swapped is None or heater_id not in swapped[1]


def test_trim_keeps_the_stripe_under_the_bus_pad(electrical_benchmark_result):
    """Milestone 1 part 2 (eight-row bundles plan): ``route_electrical_heaters``
    re-runs ``trim_common_bus_to_connections`` once the pad plan exists, with
    ``keep_span_to_x_um`` set to the outer edge of the escape's band under
    the common-bus pad slot (``pad_slots.common_bus_stripe_reach_x_um``,
    2026-10-05: the pad centre left half the 400 um band hanging past the
    stripe's end, a step in the outer corner), so the trimmed stripe reaches
    under that pad instead of stopping at the last branch -- the escape then
    rises out of the stripe flush with its end instead of doglegging in from
    further away.

    The escape still lands one grid cell (routing_grid_pitch_um, 10 um) off
    dead-straight here: ``_choose_dogleg_target`` picks the horizontal
    midpoint of the pad's target-cell column range, and the stripe's
    half-overlap widening (also one grid cell in this configuration) can
    land the stripe's own endpoint on the adjacent cell instead of that
    exact midpoint column. That is a residual quantization gap in
    ``escape_router``, not a corridor conflict: the corner sits right next
    to the bus, and the rest of the run to the pad is a straight vertical.
    """

    result = electrical_benchmark_result("heater_s_mod")

    pad_plan = result.pad_plan
    assert pad_plan is not None
    common_bus_assignment = pad_plan.common_bus_assignment
    assert common_bus_assignment is not None
    bus_pad_slot_x_um = common_bus_assignment.slot.center[0]

    bus_min_x_um, _, bus_max_x_um, _ = result.common_bus.bus.bbox
    assert bus_min_x_um <= bus_pad_slot_x_um <= bus_max_x_um

    escape = result.common_bus_escape
    assert escape is not None
    assert escape.success
    # the escape's realized band ends flush with the stripe's end: no step
    # in the outer corner (bus pad on the right by default)
    from translation.electrical.metal_realization import common_bus_escape_rects

    config = ElectricalRoutingConfig()
    escape_rects = common_bus_escape_rects(result.common_bus, escape, result.obstacle_map, config)
    assert escape_rects
    assert max(rect[2] for rect in escape_rects) == pytest.approx(bus_max_x_um)
    assert _path_direction_changes(escape.path) <= 1


def test_trim_reaches_the_escape_band_on_the_left_when_the_bus_pad_is_on_the_left():
    """``common_bus_stripe_reach_x_um`` mirrors for ``common_bus_pad_position="left"``:
    the stripe reaches the band's outer (left) edge and the escape rises flush
    from the stripe's start (merge review 2026-10-05: the left branch had no test).
    """

    from translation.electrical.metal_realization import common_bus_escape_rects
    from translation.electrical.pad_slots import common_bus_stripe_reach_x_um

    schematic = build_single_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(pad_side="top", common_bus_pad_position="left")

    result = route_electrical_heaters(component, schematic, config)

    assert result.verification is not None
    assert result.verification.success, result.verification.issues
    pad_plan = result.pad_plan
    assert pad_plan is not None and pad_plan.common_bus_assignment is not None
    reach_x_um = common_bus_stripe_reach_x_um(pad_plan, config)
    centre_x_um = pad_plan.common_bus_assignment.slot.center[0]
    assert reach_x_um == pytest.approx(centre_x_um - config.bus_width_um / 2.0)
    bus_min_x_um, _, _, _ = result.common_bus.bus.bbox
    assert bus_min_x_um == pytest.approx(reach_x_um)
    escape_rects = common_bus_escape_rects(
        result.common_bus, result.common_bus_escape, result.obstacle_map, config
    )
    assert escape_rects
    assert min(rect[0] for rect in escape_rects) == pytest.approx(bus_min_x_um)


# Also marked e2e (routes a full 8-mode cascade benchmark end to end).
@pytest.mark.integration
@pytest.mark.e2e
def test_lone_heaters_drop_straight_to_the_stripe_before_the_greedy_phase(
    electrical_benchmark_result,
):
    """Phase 2b of ``route_common_bus`` (``_route_lone_straight_drops``,
    compact-diagonal plan 2026-10-05): on the compact 8-mode diagonal every
    cascade MZI heater stands alone in its column and row, so neither trunk
    phase serves it; its bus branch is a straight drop -- a stub and one
    vertical run ending on the stripe, never a sideways join onto another
    heater's branch (which the greedy phase produced at a 900 um pitch).
    """

    result = electrical_benchmark_result("cascade_8_compact")

    assert result.verification is not None
    assert result.verification.success, result.verification.issues
    stripe_cells = set(result.common_bus.bus.cells)
    lone = [
        route
        for route in result.common_bus.routes
        if route.heater_id.startswith("heater_extra_")
        and int(route.heater_id.removeprefix("heater_extra_")) < 5  # the five cascade MZIs
    ]
    assert len(lone) == 5
    for route in lone:
        assert route.path[-1] in stripe_cells, route.heater_id
        assert _path_direction_changes(route.path) <= 1, route.heater_id
