import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import os
import webbrowser

import gdsfactory as gf
import pytest
from gdsfactory.gpdk import get_generic_pdk

from photonic_router.benchmark_extractor import ExtractedBenchmark, Port, extract_benchmark
from photonic_router.static_obstacle_builder import (
    GridSpec,
    StaticObstacleMapConfig,
    _build_static_obstacle_map_rust,
    build_static_obstacle_map_python_from_extracted,
    build_port_open_cells,
    build_static_obstacle_map,
    expand_bbox,
    grid_cell_center,
    grid_to_svg,
    inflate_cells,
    make_grid_spec,
    physical_to_grid,
    rasterize_polygon,
)

get_generic_pdk().activate()


def should_show_svg_popup() -> bool:
    """Open debug SVGs for IDE runs or when explicitly requested."""

    return (
        os.environ.get("SHOW_SVG") == "1"
        or os.environ.get("PYCHARM_HOSTED") == "1"
    )


def test_static_obstacle_map_config_defaults_are_strict_bounding_boxes():
    cfg = StaticObstacleMapConfig()
    assert cfg.obstacle_mode == "bounding_boxes"
    assert cfg.clear_port_open_cells_from_static is False


def test_extracts_geometry_and_reference_ports_from_component():
    component = gf.Component("extract_test")
    rect = component << gf.components.rectangle(size=(2.0, 1.0), layer=(1, 0))
    rect.name = "rect0"
    straight = component << gf.components.straight(length=5.0)
    straight.name = "wg0"
    straight.movey(3.0)

    extracted = extract_benchmark(component)

    assert extracted.polygons
    assert extracted.bbox[0] <= 0.0
    assert extracted.bbox[1] <= 0.0
    assert extracted.bbox[2] >= 5.0
    assert extracted.bbox[3] >= 3.0
    assert any(port.name.endswith(",o1") for port in extracted.ports)
    assert any(port.name.endswith(",o2") for port in extracted.ports)
    assert any(port.position == (0.0, 3.0) for port in extracted.ports)


def test_extract_benchmark_can_use_polygon_bounding_boxes():
    component = gf.Component("extract_bbox_test")
    component.add_polygon([(0.2, 0.4), (2.8, 0.7), (1.1, 2.6)], layer=(1, 0))

    exact = extract_benchmark(component, layers=((1, 0),))
    bbox = extract_benchmark(component, layers=((1, 0),), as_bounding_boxes=True)

    assert len(exact.polygons) == 1
    assert len(bbox.polygons) == 1
    assert [coord for point in bbox.polygons[0] for coord in point] == pytest.approx(
        [0.2, 0.4, 2.8, 0.4, 2.8, 2.6, 0.2, 2.6]
    )
    assert bbox.bbox == exact.bbox


def test_die_size_computation_manual_and_automatic():
    benchmark = ExtractedBenchmark(
        polygons=[[(1.0, 2.0), (3.0, 2.0), (3.0, 4.0), (1.0, 4.0)]],
        ports=[],
        bbox=(1.0, 2.0, 3.0, 4.0),
    )

    assert expand_bbox(benchmark.bbox, 2.0) == (-1.0, 0.0, 5.0, 6.0)

    auto_grid = make_grid_spec(
        benchmark,
        grid_size_um=0.5,
        security_margin_um=1.0,
    )
    assert auto_grid.die_bbox == (0.0, 1.0, 4.0, 5.0)
    assert auto_grid.width == 8
    assert auto_grid.height == 8

    padded_grid = make_grid_spec(
        benchmark,
        grid_size_um=0.5,
        security_margin_um=1.0,
        chip_add_x_um=2.0,
        chip_add_y_um=3.0,
    )
    assert padded_grid.die_bbox == (-2.0, -2.0, 6.0, 8.0)
    assert padded_grid.width == 16
    assert padded_grid.height == 20

    manual_grid = make_grid_spec(
        benchmark,
        grid_size_um=0.5,
        security_margin_um=1.0,
        chip_add_x_um=2.0,
        chip_add_y_um=3.0,
        die_bbox=(-10.0, -5.0, 0.0, 5.0),
    )
    assert manual_grid.die_bbox == (-10.0, -5.0, 0.0, 5.0)
    assert manual_grid.width == 20
    assert manual_grid.height == 20


def test_coordinate_transformation_and_cell_center():
    grid = GridSpec(
        width=10,
        height=10,
        grid_size_um=0.5,
        origin=(-1.0, 2.0),
        die_bbox=(-1.0, 2.0, 4.0, 7.0),
    )

    assert physical_to_grid(-1.0, 2.0, grid) == (0, 0)
    assert physical_to_grid(-0.51, 2.49, grid) == (0, 0)
    assert physical_to_grid(0.0, 3.0, grid) == (2, 2)
    assert grid_cell_center(2, 2, grid) == (0.25, 3.25)


def test_rasterizes_polygon_by_cell_centers():
    grid = GridSpec(
        width=2,
        height=2,
        grid_size_um=1.0,
        origin=(0.0, 0.0),
        die_bbox=(0.0, 0.0, 2.0, 2.0),
    )
    polygon = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]

    assert rasterize_polygon(polygon, grid) == {(0, 0), (1, 0), (0, 1), (1, 1)}


def test_clearance_expansion_manhattan_and_chebyshev():
    manhattan = inflate_cells({(2, 2)}, 5, 5, 1, metric="manhattan")
    chebyshev = inflate_cells({(2, 2)}, 5, 5, 1, metric="chebyshev")

    assert manhattan == {(2, 2), (1, 2), (3, 2), (2, 1), (2, 3)}
    assert chebyshev == {
        (1, 1),
        (2, 1),
        (3, 1),
        (1, 2),
        (2, 2),
        (3, 2),
        (1, 3),
        (2, 3),
        (3, 3),
    }


def test_port_opening_generation():
    benchmark = ExtractedBenchmark(
        polygons=[],
        ports=[Port(name="o1", position=(1.0, 1.0), orientation=0.0)],
        bbox=(0.0, 0.0, 3.0, 3.0),
    )
    grid = GridSpec(
        width=3,
        height=3,
        grid_size_um=1.0,
        origin=(0.0, 0.0),
        die_bbox=(0.0, 0.0, 3.0, 3.0),
    )

    assert build_port_open_cells(benchmark, grid, radius=1) == {
        (0, 0),
        (1, 0),
        (2, 0),
        (0, 1),
        (1, 1),
        (2, 1),
        (0, 2),
        (1, 2),
        (2, 2),
    }


def test_build_static_obstacle_map_and_debug_svg(tmp_path):
    component = gf.Component("builder_test")
    component.add_polygon(
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
        layer=(1, 0),
    )

    component.add_polygon(
        [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)],
        layer=(1, 0),
    )

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            die_bbox=(0.0, 0.0, 15.0, 15.0),
        ),
    )

    # assert data.grid.width == 2
    # assert data.grid.height == 2
    # assert data.raw_blocked_cells == {(0, 0), (1, 0), (0, 1), (1, 1)}
    # assert data.rust_static_cells() == [(0, 0), (0, 1), (1, 0), (1, 1)]

    svg = grid_to_svg(data.grid, data.blocked_cells, {(1, 1)})
    assert "<svg" in svg
    assert 'class="port-access"' in svg
    assert 'fill="#d93025" opacity="0.38"' in svg
    # assert "#202124" in svg

    debug_path = tmp_path / "obstacles.svg"
    data.export_debug_svg(debug_path)
    assert debug_path.exists()

    if should_show_svg_popup():
        webbrowser.open_new_tab(debug_path.as_uri())


def test_build_static_obstacle_map_can_filter_obstacle_layers():
    component = gf.Component("layer_filter_test")
    component.add_polygon(
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
        layer=(1, 0),
    )
    component.add_polygon(
        [(10.0, 10.0), (12.0, 10.0), (12.0, 12.0), (10.0, 12.0)],
        layer=(49, 0),
    )

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 15.0, 15.0),
            obstacle_layers=((1, 0),),
        ),
    )

    # Optical layer cells are blocked.
    assert (0, 0) in data.blocked_cells
    assert (1, 1) in data.blocked_cells
    # Non-optical layer cells are ignored.
    assert (10, 10) not in data.blocked_cells
    assert (11, 11) not in data.blocked_cells


def test_build_static_obstacle_map_bounding_box_mode_blocks_polygon_bbox():
    component = gf.Component("bbox_builder_test")
    component.add_polygon(
        [(0.1, 0.2), (2.9, 0.2), (2.9, 1.8), (0.1, 1.8)],
        layer=(1, 0),
    )

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            obstacle_mode="bounding_boxes",
            die_bbox=(0.0, 0.0, 4.0, 4.0),
        ),
    )

    assert data.blocked_cells == {(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)}


def test_build_static_obstacle_map_bounding_box_mode_conservative_for_triangle():
    component = gf.Component("bbox_triangle_test")
    component.add_polygon(
        [(0.0, 0.0), (2.8, 0.0), (0.0, 2.8)],
        layer=(1, 0),
    )

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            obstacle_mode="bounding_boxes",
            die_bbox=(0.0, 0.0, 4.0, 4.0),
        ),
    )

    assert data.blocked_cells == {(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1), (0, 2), (1, 2), (2, 2)}


def test_build_static_obstacle_map_bounding_box_mode_clearance_expands_bbox():
    component = gf.Component("bbox_clearance_test")
    component.add_polygon(
        [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)],
        layer=(1, 0),
    )

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=1.0,
            obstacle_mode="bounding_boxes",
            die_bbox=(0.0, 0.0, 5.0, 5.0),
        ),
    )

    assert (0, 0) in data.blocked_cells
    assert (0, 1) in data.blocked_cells
    assert (0, 2) in data.blocked_cells
    assert (2, 2) in data.blocked_cells
    assert (2, 0) in data.blocked_cells
    assert (3, 0) not in data.blocked_cells


def test_static_obstacle_map_can_use_separate_heater_clearance():
    component = gf.Component("split_heater_clearance")
    component.add_polygon(
        [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)],
        layer=(1, 0),
    )
    component.add_polygon(
        [(5.0, 1.0), (9.0, 1.0), (9.0, 3.0), (5.0, 3.0)],
        layer=(47, 0),
    )

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            heater_clearance_um=1.0,
            port_open_radius_um=0.0,
            obstacle_mode="bounding_boxes",
            die_bbox=(0.0, 0.0, 10.0, 4.0),
            obstacle_layers=((1, 0), (47, 0)),
            heater_obstacle_layers=((47, 0),),
        ),
    )

    assert (1, 1) in data.blocked_cells
    assert (0, 1) not in data.blocked_cells
    assert (5, 1) in data.blocked_cells
    assert (4, 1) not in data.blocked_cells
    assert (5, 0) in data.blocked_cells
    assert (5, 3) in data.blocked_cells
    assert (4, 1, 9, 3) not in data.blocked_static_rects
    assert (5, 0, 8, 3) in data.blocked_static_rects
    assert data.backend.endswith("-split")


def test_bounding_box_mode_preserves_port_opening_behavior():
    component = gf.Component("bbox_port_opening_test")
    component.add_polygon(
        [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)],
        layer=(1, 0),
    )
    component.add_port(name="o1", center=(1.5, 1.5), width=0.5, orientation=0.0, layer=(1, 0))

    config = StaticObstacleMapConfig(
        grid_size_um=1.0,
        security_margin_um=0.0,
        clearance_um=0.0,
        port_open_radius_um=0.0,
        obstacle_mode="bounding_boxes",
        clear_port_open_cells_from_static=True,
        die_bbox=(0.0, 0.0, 4.0, 4.0),
    )
    box_data = build_static_obstacle_map(component, config)

    raster_data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            obstacle_mode="rasterized_polygons",
            clear_port_open_cells_from_static=True,
            die_bbox=(0.0, 0.0, 4.0, 4.0),
        ),
    )

    assert box_data.port_open_cells == {(1, 1)}
    assert box_data.port_open_cells == raster_data.port_open_cells
    assert (1, 1) not in box_data.blocked_cells
    assert (1, 1) not in raster_data.blocked_cells


def test_rasterized_mode_can_keep_port_cells_blocked_when_disabled():
    component = gf.Component("strict_ports_rasterized")
    component.add_polygon(
        [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)],
        layer=(1, 0),
    )
    component.add_port(name="o1", center=(1.5, 1.5), width=0.5, orientation=0.0, layer=(1, 0))

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            obstacle_mode="rasterized_polygons",
            clear_port_open_cells_from_static=False,
            die_bbox=(0.0, 0.0, 4.0, 4.0),
        ),
    )

    assert (1, 1) in data.port_open_cells
    assert (1, 1) in data.blocked_cells
    assert data.raw_blocked_cells == data.blocked_cells


def test_bounding_box_mode_can_keep_port_cells_blocked_when_disabled():
    component = gf.Component("strict_ports_bounding_box")
    component.add_polygon(
        [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)],
        layer=(1, 0),
    )
    component.add_port(name="o1", center=(1.5, 1.5), width=0.5, orientation=0.0, layer=(1, 0))

    data = build_static_obstacle_map(
        component,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            obstacle_mode="bounding_boxes",
            clear_port_open_cells_from_static=False,
            die_bbox=(0.0, 0.0, 4.0, 4.0),
        ),
    )

    assert (1, 1) in data.port_open_cells
    assert (1, 1) in data.blocked_cells
    assert (1, 1) in data.raw_blocked_cells


def test_clear_port_open_cells_default_is_strict():
    benchmark = ExtractedBenchmark(
        polygons=[[(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]],
        ports=[Port(name="o1", position=(1.5, 1.5), orientation=0.0)],
        bbox=(1.0, 1.0, 3.0, 3.0),
    )

    data = build_static_obstacle_map_python_from_extracted(
        benchmark,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            die_bbox=(0.0, 0.0, 4.0, 4.0),
        ),
    )

    assert (1, 1) in data.port_open_cells
    assert (1, 1) in data.blocked_cells


def test_rust_obstacle_builder_compat_with_legacy_signature_keeps_default_behavior():
    benchmark = ExtractedBenchmark(
        polygons=[[(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]],
        ports=[Port(name="o1", position=(1.0, 1.0), orientation=0.0)],
        bbox=(0.0, 0.0, 3.0, 3.0),
    )

    class _LegacyRustBackend:
        def build_static_obstacle_map_rs(self, *args):
            # Simulate older extension signatures without the strict-port flag.
            if len(args) in (9, 10):
                raise TypeError(
                    "build_static_obstacle_map_rs() takes from 7 to 8 positional arguments"
                    f" but {len(args)} were given"
                )
            if len(args) == 8:
                return {
                    "grid": (3, 3, 1.0, (0.0, 0.0), (0.0, 0.0, 3.0, 3.0)),
                    "raw_blocked_cells": [(0, 0), (1, 0), (0, 1), (1, 1), (2, 1), (1, 2)],
                    "blocked_cells": [(0, 0), (1, 0), (0, 1), (2, 1), (1, 2)],
                    "port_open_cells": [(1, 1)],
                    "stats": {
                        "blocked_cells_after_port_opening": 5,
                        "raw_blocked_cell_count": 6,
                    },
                }
            raise TypeError(f"unexpected argument count {len(args)}")

    backend = _LegacyRustBackend()

    default_data = _build_static_obstacle_map_rust(
        benchmark,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            obstacle_mode="rasterized_polygons",
            die_bbox=(0.0, 0.0, 3.0, 3.0),
        ),
        backend,
    )

    strict_data = _build_static_obstacle_map_rust(
        benchmark,
        StaticObstacleMapConfig(
            grid_size_um=1.0,
            security_margin_um=0.0,
            clearance_um=0.0,
            port_open_radius_um=0.0,
            obstacle_mode="rasterized_polygons",
            clear_port_open_cells_from_static=False,
            die_bbox=(0.0, 0.0, 3.0, 3.0),
        ),
        backend,
    )

    assert (1, 1) in default_data.blocked_cells
    assert (1, 1) in strict_data.blocked_cells
