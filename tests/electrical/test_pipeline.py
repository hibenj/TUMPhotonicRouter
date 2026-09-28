"""Tests for translation/electrical/route_electrical.py and stages.py.

The twelve-stage orchestration itself: the no-op guard when a layout has no
heater terminals, and that every stage implementation still matches its
``stages.py`` Protocol shape.
"""

from __future__ import annotations

from tests.electrical.support import _polygon_bboxes_by_layer
from tests.fixtures.synthetic_layouts import (
    path_length_schematic as build_heaterless_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters, stages
from translation.electrical.bundle_detail_router import route_detailed_bundles
from translation.electrical.common_bus_router import (
    route_common_bus,
    trim_common_bus_to_connections,
)
from translation.electrical.debug import write_debug_artifacts
from translation.electrical.escape_router import route_common_bus_escape
from translation.electrical.individual_topology import compute_individual_escape_topology
from translation.electrical.metal_realization import realize_electrical_metal
from translation.electrical.obstacle_extraction import build_electrical_obstacle_map
from translation.electrical.pad_side_reconciliation import reconcile_pad_sides
from translation.electrical.pad_slots import plan_pad_slots
from translation.electrical.terminal_extraction import extract_heater_terminal_pairs
from translation.electrical.verification import verify_electrical_routing
from translation.layout_from_schematic import layout_from_schematic


def test_electrical_routing_is_noop_without_heater_terminals(tmp_path):
    schematic = build_heaterless_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig()
    before_metal = _polygon_bboxes_by_layer(component, config.metal_layer)

    result = route_electrical_heaters(
        component,
        schematic,
        config,
        debug_dir=tmp_path,
        debug_prefix="toy",
    )

    assert result.terminal_groups == ()
    assert result.common_bus.success
    assert result.common_bus.routes == ()
    assert result.common_bus.tree_cells == frozenset()
    assert result.pad_plan is None
    assert result.common_bus_escape is None
    assert result.detailed_bundle_routes is None
    assert result.verification is None
    assert result.debug_artifacts == {}
    assert result.routed_component is not None
    assert _polygon_bboxes_by_layer(result.routed_component, config.metal_layer) == before_metal
    assert config.pad_marker_layer is not None
    assert not _polygon_bboxes_by_layer(result.routed_component, config.pad_marker_layer)
    assert "electrical_metal_realization" not in result.routed_component.info
    assert not (tmp_path / "electrical" / "toy_common_bus.svg").exists()


def test_electrical_stage_functions_satisfy_their_protocols() -> None:
    """Each stage implementation's signature matches its `stages.py` Protocol.

    A static-shape test: binding an implementation to a Protocol-annotated
    variable is what a type checker (mypy) verifies against the Protocol's
    `__call__`; this test only asserts every stage is still a plain callable,
    so a stage accidentally turned into something uncallable is caught even
    without running mypy.
    """

    terminal_extractor: stages.TerminalExtractor = extract_heater_terminal_pairs
    obstacle_map_builder: stages.ObstacleMapBuilder = build_electrical_obstacle_map
    common_bus_router: stages.CommonBusRouter = route_common_bus
    common_bus_trimmer: stages.CommonBusTrimmer = trim_common_bus_to_connections
    escape_topology_planner: stages.EscapeTopologyPlanner = compute_individual_escape_topology
    pad_planner: stages.PadPlanner = plan_pad_slots
    pad_side_reconciler: stages.PadSideReconciler = reconcile_pad_sides
    bus_escape_router: stages.BusEscapeRouter = route_common_bus_escape
    pad_wire_router: stages.PadWireRouter = route_detailed_bundles
    metal_realizer: stages.MetalRealizer = realize_electrical_metal
    electrical_verifier: stages.ElectricalVerifier = verify_electrical_routing
    debug_artifact_writer: stages.DebugArtifactWriter = write_debug_artifacts

    for stage_fn in (
        terminal_extractor,
        obstacle_map_builder,
        common_bus_router,
        common_bus_trimmer,
        escape_topology_planner,
        pad_planner,
        pad_side_reconciler,
        bus_escape_router,
        pad_wire_router,
        metal_realizer,
        electrical_verifier,
        debug_artifact_writer,
    ):
        assert callable(stage_fn)
