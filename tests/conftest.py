"""Marker application and shared fixtures for the test suite.

The marker application (ExecPlan Milestone 6, Slice 1) only attaches the
``e2e`` and ``integration`` markers registered in ``pyproject.toml`` so the
suite can be split by kind (``pytest -m "not e2e"`` etc.) without moving or
editing the tests that already live outside ``tests/e2e/``.

The fixtures below (Slice 2) expose the synthetic layout/schematic/session
builders in ``tests/fixtures/`` as pytest fixtures. A test file may take one
of these fixtures, or import the builder directly from ``tests.fixtures``
when the parameters it varies make a bare import simpler than a fixture
parameter; either way, ``tests/fixtures/`` is the only place these synthetic
layouts are built.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from tests.fixtures.sessions import Pipeline, pipeline_for_test
from tests.fixtures.sessions import (
    route_electrical_benchmark_result as _route_electrical_benchmark_result,
)
from tests.fixtures.sessions import settings_for_test as _settings_for_test
from tests.fixtures.synthetic_layouts import (
    DummySchematic,
    GRID_HEIGHT,
    GRID_WIDTH,
    port_from_instance,
    single_rectangle_schematic as _single_rectangle_schematic,
    three_net_schematic,
)

# Mixed files: tests that load a benchmark and route it, or spawn a
# subprocess, named by node id because the rest of their file is ordinary
# unit-level testing.
_E2E_NODE_IDS = {
    "tests/electrical/test_metal_realization.py::test_show_realized_electrical_metal_in_klayout",
    "tests/test_port_alignment_diagnostics.py::test_heater_s_mod_pass0_characterizes_current_port_alignment",
    "tests/test_port_alignment_diagnostics.py::test_heater_s_mod_route_match_uses_corrected_records_for_realization",
    "tests/test_route_rust_geometry.py::test_rust_routed_layout_uses_waveguide_geometry",
}

# Integration tests named by node id, the same mechanism as `_E2E_NODE_IDS`
# above: `tests/test_electrical_routing.py` ran the whole electrical
# pipeline on the mmi_heater toy benchmarks until Milestone 6 Slice 3 moved
# it onto the synthetic heater layouts in `tests/fixtures/`. It no longer
# loads a benchmark, so these are integration, not end-to-end; the one test
# that spawns KLayout stays in `_E2E_NODE_IDS`. Milestone 4 Slice 5 split
# that one file into `tests/electrical/`, one module per
# `translation/electrical/stages.py` stage; each module still holds pure
# unit tests alongside these, so the classification stays per test (by node
# id, now under its new path). Two node ids from the pre-split list matched
# no test at HEAD (a stale rename each: the old
# `test_common_bus_local_trunk_strategy_creates_middle_trunk_for_pairs` and
# `test_detailed_bundle_router_assigns_spaced_offsets_from_topology`, the
# latter now `test_detailed_bundle_router_nests_l_and_z_pad_wires_from_topology`
# in `test_pad_wires.py`); they were already dead and are dropped here rather
# than carried forward, since restoring the marker they never actually
# applied would change this slice's pass/marker count, which must stay
# unchanged except for the one new integration test below.
_INTEGRATION_NODE_IDS = {
    "tests/electrical/test_terminal_extraction.py::test_extracts_two_logical_terminals_from_multi_port_heater",
    "tests/electrical/test_obstacle_map.py::test_obstacle_map_uses_role_specific_terminal_openings",
    "tests/electrical/test_obstacle_map.py::test_common_bus_rail_extends_toward_common_bus_pad_side",
    "tests/electrical/test_obstacle_map.py::test_common_bus_rail_and_pad_escape_use_bus_width_only",
    "tests/electrical/test_pipeline.py::test_electrical_routing_is_noop_without_heater_terminals",
    "tests/electrical/test_common_bus.py::test_common_bus_router_selects_exactly_one_terminal_per_heater",
    "tests/electrical/test_common_bus.py::test_common_bus_terminal_selection_prefers_local_same_row_pair_midpoint",
    "tests/electrical/test_common_bus.py::test_pad_side_reconciliation_swaps_a_lone_heater_and_drops_its_branch_straight",
    "tests/electrical/test_pad_slots.py::test_pad_plan_assigns_slots_without_realizing_geometry",
    "tests/electrical/test_pad_slots.py::test_pad_access_is_only_on_chip_facing_pad_edge",
    "tests/electrical/test_pad_slots.py::test_top_pad_offset_moves_pad_row_further_from_layout",
    "tests/electrical/test_pad_slots.py::test_pad_plan_allows_empty_pitch_slots_and_keeps_individual_order",
    "tests/electrical/test_pad_slots.py::test_auto_pad_origin_compacts_row_toward_escape_topology",
    "tests/electrical/test_pad_slots.py::test_auto_pad_channel_height_uses_widest_topology_bundle",
    "tests/electrical/test_pad_slots.py::test_auto_pad_assignment_places_topology_bundles_as_intervals_with_gaps",
    "tests/electrical/test_individual_topology.py::test_individual_topology_groups_escape_corridors_before_pad_assignment",
    "tests/electrical/test_metal_realization.py::test_metal_realization_creates_assigned_pads_but_not_empty_slots",
    "tests/electrical/test_metal_realization.py::test_metal_realization_adds_wire_polygons_for_bus_and_individual_routes",
    "tests/electrical/test_escape_router.py::test_common_bus_escape_reaches_assigned_common_bus_pad_slot",
    "tests/electrical/test_escape_router.py::test_common_bus_escape_uses_opposite_bus_for_bottom_pad_side",
}

# Integration files: several stages or the Rust router on a synthetic
# layout, or benchmark schematics without routing.
_INTEGRATION_FILE_STEMS = {
    "test_preplaced_crossing_grids",
    "test_crossing_structures",
    "test_benes_benchmark",
    "test_graph_analysis_context",
    "test_clements_benchmark_alignment",
    "test_route_rust_opened_cells",
    "test_routing_stages",
    "test_static_obstacle_builder",
}


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    rootdir = Path(str(config.rootdir))
    for item in items:
        try:
            rel_path = Path(str(item.fspath)).resolve().relative_to(rootdir.resolve())
        except ValueError:
            rel_path = Path(str(item.fspath))
        rel_posix = rel_path.as_posix()

        if rel_path.parts[:2] == ("tests", "e2e"):
            item.add_marker(pytest.mark.e2e)
            continue

        node_id = f"{rel_posix}::{item.name}"
        if node_id in _E2E_NODE_IDS:
            item.add_marker(pytest.mark.e2e)
            continue

        if node_id in _INTEGRATION_NODE_IDS or rel_path.stem in _INTEGRATION_FILE_STEMS:
            item.add_marker(pytest.mark.integration)


@pytest.fixture
def single_rectangle_schematic():
    """Factory: `single_rectangle_schematic(placement)` builds a one-instance
    schematic (a 10x2 rectangle at `placement`); see
    `tests/test_layout_from_schematic.py`."""
    return _single_rectangle_schematic


@pytest.fixture
def dummy_schematic():
    """Factory: the `DummySchematic` dataclass used by
    `tests/test_route_rust_opened_cells.py`, e.g.
    `dummy_schematic(netlist=DummyNetlist(routes={...}))`."""
    return DummySchematic


@pytest.fixture
def three_net_layout() -> Any:
    """The three-net synthetic scenario `tests/test_routing_stages.py` and the
    `pipeline` fixture below run the routing stages over: a `left` instance
    fanning out to three `right*` grating couplers on a 30x20 one-micron grid."""
    return SimpleNamespace(
        schematic=three_net_schematic(),
        port_from_instance=port_from_instance,
        grid_width=GRID_WIDTH,
        grid_height=GRID_HEIGHT,
    )


@pytest.fixture
def settings_for_test():
    """Factory: `settings_for_test(**overrides)` builds a `SessionSettings`
    for a session assembled with `object.__new__`; see
    `tests.fixtures.sessions.settings_for_test`."""
    return _settings_for_test


@pytest.fixture
def pipeline(monkeypatch: pytest.MonkeyPatch) -> Pipeline:
    """The nine `translation.routing` stages over the three-net synthetic
    scenario, each run at most once; see `tests.fixtures.sessions.Pipeline`."""
    return pipeline_for_test(monkeypatch)


@pytest.fixture
def electrical_benchmark_result():
    """Factory: `electrical_benchmark_result(benchmark_name)` routes a named
    benchmark through the full electrical pipeline with a default
    `ElectricalRoutingConfig`; see
    `tests.fixtures.sessions.route_electrical_benchmark_result`."""
    return _route_electrical_benchmark_result
