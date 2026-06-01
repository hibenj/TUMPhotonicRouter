import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import os
import webbrowser

import gdsfactory as gf

from photonic_router.benchmark_extractor import ExtractedBenchmark, Port, extract_benchmark
from photonic_router.static_obstacle_builder import (
    GridSpec,
    StaticObstacleMapConfig,
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


def should_show_svg_popup() -> bool:
    """Open debug SVGs for IDE runs or when explicitly requested."""

    return (
        os.environ.get("SHOW_SVG") == "1"
        or os.environ.get("PYCHARM_HOSTED") == "1"
    )


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

    manual_grid = make_grid_spec(
        benchmark,
        grid_size_um=0.5,
        security_margin_um=1.0,
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

    svg = grid_to_svg(data.grid, data.blocked_cells)
    assert "<svg" in svg
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
