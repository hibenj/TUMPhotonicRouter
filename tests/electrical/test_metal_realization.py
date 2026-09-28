"""Tests for translation/electrical/metal_realization.py and rect_geometry.py.

Stage 10 (``MetalRealizer``): realizing every routed path as metal polygons
on the layout, and the rectangle-union primitives it builds on.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

from tests.electrical.support import _bbox_contains, _polygon_region_by_layer, _region_covers_bbox
from tests.fixtures.synthetic_layouts import (
    multi_heater_schematic as build_multi_heater_schematic,
)
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.electrical.rect_geometry import clean_rects, union_rect_area, wire_rects_for_points
from translation.layout_from_schematic import layout_from_schematic

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def test_wire_rect_generation_omits_redundant_vertex_squares():
    rects = wire_rects_for_points(
        ((0.0, 0.0), (20.0, 0.0), (20.0, 20.0)),
        width_um=10.0,
    )

    assert rects == (
        (-5.0, -5.0, 15.0, 5.0),
        (15.0, -5.0, 25.0, 25.0),
    )
    assert clean_rects(
        (
            rects[0],
            rects[0],
            (0.0, -2.0, 10.0, 2.0),
            (25.0, -5.0, 35.0, 5.0),
        )
    ) == (
        (-5.0, -5.0, 15.0, 5.0),
        (25.0, -5.0, 35.0, 5.0),
    )
    assert union_rect_area(rects) == 500.0


def test_metal_realization_creates_assigned_pads_but_not_empty_slots():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=None,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    routed_component = result.routed_component
    assert routed_component is not None
    metal_region = _polygon_region_by_layer(routed_component, config.metal_layer)
    assert config.pad_marker_layer is not None
    marker_region = _polygon_region_by_layer(routed_component, config.pad_marker_layer)

    assert result.pad_plan.assignments
    assigned_bboxes = tuple(assignment.slot.bbox for assignment in result.pad_plan.assignments)
    for assignment in result.pad_plan.assignments:
        assert _region_covers_bbox(metal_region, assignment.slot.bbox)
        assert _region_covers_bbox(marker_region, assignment.slot.bbox)
    for slot in result.pad_plan.empty_slots:
        if any(_bbox_contains(assigned_bbox, slot.bbox) for assigned_bbox in assigned_bboxes):
            continue
        assert not _region_covers_bbox(marker_region, slot.bbox)


def test_metal_realization_adds_wire_polygons_for_bus_and_individual_routes():
    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=None,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.routed_component is not None
    assert result.verification is not None
    realization_metrics = dict(result.routed_component.info["electrical_metal_realization"])
    polygons = result.routed_component.get_polygons(
        merge=False,
        by="tuple",
    ).get(config.metal_layer, [])
    pre_union_rect_count = result.verification.metrics["rect_count"]
    assert polygons
    assert len(polygons) < pre_union_rect_count
    assert realization_metrics["metal_area_overcount_um2"] == pytest.approx(0.0, abs=1e-6)
    assert realization_metrics["raw_metal_area_um2"] == pytest.approx(
        realization_metrics["union_metal_area_um2"]
    )
    assert realization_metrics["pre_union_rect_count"] >= pre_union_rect_count
    assert realization_metrics["output_polygon_count"] < realization_metrics["pre_union_rect_count"]
    assert realization_metrics["output_polygon_count"] <= len(polygons)
    assert len(polygons) >= len(result.detailed_bundle_routes.routes)


def test_show_realized_electrical_metal_in_klayout():
    if os.environ.get("SHOW_ELECTRICAL_KLAYOUT") != "1":
        pytest.skip("set SHOW_ELECTRICAL_KLAYOUT=1 to open KLayout")

    schematic = build_multi_heater_schematic()
    component = layout_from_schematic(schematic)
    config = ElectricalRoutingConfig(
        pad_side="top",
        pad_pitch_um=150.0,
        pad_origin_x_um=None,
        pad_offset_um=500.0,
        routing_grid_pitch_um=20.0,
        obstacle_clearance_um=0.0,
        terminal_open_radius_um=20.0,
        wire_width_um=20.0,
        individual_route_spacing_um=20.0,
    )

    result = route_electrical_heaters(component, schematic, config)

    assert result.routed_component is not None
    assert result.detailed_bundle_routes.success
    result.routed_component.show()
    gds_path = PROJECT_ROOT / "build" / "electrical" / "realized_electrical_metal.gds"
    gds_path.parent.mkdir(parents=True, exist_ok=True)
    result.routed_component.write_gds(gds_path)
    subprocess.Popen(
        ["klayout", str(gds_path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
