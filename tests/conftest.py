"""Marker application for the test suite (ExecPlan Milestone 6, Slice 1).

No test body changes here: this module only attaches the ``e2e`` and
``integration`` markers registered in ``pyproject.toml`` so the suite can be
split by kind (``pytest -m "not e2e"`` etc.) without moving or editing the
tests that already live outside ``tests/e2e/``.
"""

from __future__ import annotations

from pathlib import Path

import pytest

# Mixed files: tests that route or import an unroutable toy benchmark
# (`benchmarks.mmi_heater*`, `benchmarks.TOY`) or spawn a subprocess, named
# by node id because the rest of their file is ordinary unit-level testing.
_E2E_NODE_IDS = {
    "tests/test_electrical_routing.py::test_extracts_two_logical_terminals_from_multi_port_heater",
    "tests/test_electrical_routing.py::test_obstacle_map_uses_role_specific_terminal_openings",
    "tests/test_electrical_routing.py::test_common_bus_rail_extends_toward_common_bus_pad_side",
    "tests/test_electrical_routing.py::test_common_bus_rail_and_pad_escape_use_bus_width_only",
    "tests/test_electrical_routing.py::test_electrical_routing_is_noop_without_heater_terminals",
    "tests/test_electrical_routing.py::test_common_bus_router_selects_exactly_one_terminal_per_heater",
    "tests/test_electrical_routing.py::test_common_bus_terminal_selection_prefers_local_same_row_pair_midpoint",
    "tests/test_electrical_routing.py::test_common_bus_local_trunk_strategy_creates_middle_trunk_for_pairs",
    "tests/test_electrical_routing.py::test_pad_plan_assigns_slots_without_realizing_geometry",
    "tests/test_electrical_routing.py::test_pad_access_is_only_on_chip_facing_pad_edge",
    "tests/test_electrical_routing.py::test_top_pad_offset_moves_pad_row_further_from_layout",
    "tests/test_electrical_routing.py::test_pad_plan_allows_empty_pitch_slots_and_keeps_individual_order",
    "tests/test_electrical_routing.py::test_auto_pad_origin_compacts_row_toward_escape_topology",
    "tests/test_electrical_routing.py::test_auto_pad_channel_height_uses_widest_topology_bundle",
    "tests/test_electrical_routing.py::test_auto_pad_assignment_places_topology_bundles_as_intervals_with_gaps",
    "tests/test_electrical_routing.py::test_individual_topology_groups_escape_corridors_before_pad_assignment",
    "tests/test_electrical_routing.py::test_detailed_bundle_router_assigns_spaced_offsets_from_topology",
    "tests/test_electrical_routing.py::test_metal_realization_creates_assigned_pads_but_not_empty_slots",
    "tests/test_electrical_routing.py::test_metal_realization_adds_wire_polygons_for_bus_and_individual_routes",
    "tests/test_electrical_routing.py::test_show_realized_electrical_metal_in_klayout",
    "tests/test_electrical_routing.py::test_common_bus_escape_reaches_assigned_common_bus_pad_slot",
    "tests/test_electrical_routing.py::test_common_bus_escape_uses_opposite_bus_for_bottom_pad_side",
    "tests/test_port_alignment_diagnostics.py::test_mmi_heater_pass0_characterizes_current_port_alignment",
    "tests/test_port_alignment_diagnostics.py::test_mmi_heater_route_match_uses_corrected_records_for_realization",
    "tests/test_route_rust_geometry.py::test_rust_routed_layout_uses_waveguide_geometry",
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

        if rel_path.stem in _INTEGRATION_FILE_STEMS:
            item.add_marker(pytest.mark.integration)
