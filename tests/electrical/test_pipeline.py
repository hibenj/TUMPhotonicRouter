"""Tests for python/photonic_router/electrical/route_electrical.py.

The twelve-stage orchestration itself: the no-op guard when a layout has no
heater terminals, and what the configuration refuses.
"""

from __future__ import annotations

import pytest
from photonic_router.electrical import ElectricalRoutingConfig, route_electrical_heaters
from photonic_router.records.layout_from_schematic import layout_from_schematic

from tests.electrical.support import _polygon_bboxes_by_layer
from tests.fixtures.synthetic_layouts import (
    path_length_schematic as build_heaterless_schematic,
)


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


def test_config_rejects_zero_obstacle_clearance() -> None:
    """`obstacle_clearance_um == 0` lets metal touch an obstacle, which the
    verifier reports as a short; `validate()` requires a positive clearance
    (ExecPlan Decision Log, 2026-09-28).
    """

    with pytest.raises(ValueError, match="obstacle_clearance_um must be positive"):
        ElectricalRoutingConfig(obstacle_clearance_um=0.0).validate()
