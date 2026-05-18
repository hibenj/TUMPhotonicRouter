"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic
from gdsfactory.typings import Port

from translation.route_gds import get_port_from_instance

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
build_static_obstacle_map = _sob.build_static_obstacle_map
physical_to_grid = _sob.physical_to_grid
_load_rust_backend = _sob._load_rust_backend


@dataclass(frozen=True)
class RustRouteDebugArtifacts:
    obstacle_svg: Path | None
    route_svgs: list[Path]


def _orientation_to_angle(orientation: float | None, *, flip: bool = False) -> int:
    if orientation is None:
        return 0
    normalized = orientation + (180.0 if flip else 0.0)
    normalized %= 360.0
    return int(round(normalized / 45.0)) % 8


def _flip_orientation(orientation: float | None) -> float | None:
    if orientation is None:
        return None
    return (float(orientation) + 180.0) % 360.0


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _grid_origin_xy(grid: GridSpec) -> tuple[float, float]:
    if hasattr(grid, "origin"):
        origin = getattr(grid, "origin")
        return float(origin[0]), float(origin[1])
    return (
        float(getattr(grid, "origin_x_um", 0.0)),
        float(getattr(grid, "origin_y_um", 0.0)),
    )


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    debug_timing: bool = False,
) -> tuple[Component, RustRouteDebugArtifacts]:
    """Route schematic nets using Rust A* and add one polygon per routed net.

    This function routes each net by:
    1. Building a static obstacle map from the unrouted layout.
    2. Calling the Rust A* router for each net.
    3. Realizing one closed polygon in Rust and inserting it with add_polygon.
    4. Updating blocked cells for subsequent nets using width-aware inflation.

    Parameters:
        unrouted_layout: Component with placed instances but no routes.
        schematic: Schematic with net definitions.
        obstacle_config: Optional obstacle-map configuration.
        debug_dir: Directory where debug SVGs are written when provided.
        debug_prefix: Prefix used for debug SVG filenames.
        route_width_um: Realized waveguide width in micrometers.
        route_layer: Target GDS layer/datatype tuple for route polygons.

    Returns:
        A tuple of (routed_layout, debug_artifacts).
        :param debug_timing:
    """
    if route_width_um <= 0:
        raise ValueError("route_width_um must be > 0")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    routed_layout = unrouted_layout.copy()
    routed_layout.name = "routed_layout_rust"

    t_obstacle_start = 0.0
    if debug_timing:
        t_obstacle_start = time.perf_counter()
    obstacle_map = build_static_obstacle_map(unrouted_layout, config=obstacle_config)
    if debug_timing:
        t_obstacle_end = time.perf_counter()
        print(f"      - Obstacle Map time: {t_obstacle_end - t_obstacle_start:.4f} s")
    grid = obstacle_map.grid
    port_open_cells = set(obstacle_map.port_open_cells)

    debug_path = Path(debug_dir) if debug_dir is not None else None
    obstacle_svg = None
    route_svgs: list[Path] = []

    if debug_path is not None:
        obstacle_dir = debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)

    nets = schematic.netlist.routes
    print(f"\nRouting {len(nets)} nets using Rust router...")

    if not hasattr(rust_backend, "PyPhotonicRouter"):
        raise RuntimeError(
            "Rust backend does not expose PyPhotonicRouter. "
            "Rebuild/install the Rust extension with the class-based API."
        )

    origin_x_um, origin_y_um = _grid_origin_xy(grid)
    grid_spec = rust_backend.GridSpec(
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        origin_x_um,
        origin_y_um,
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(grid_size_um=float(grid.grid_size_um), bend_radius_cells=3)
    astar_cfg = rust_backend.AStarConfig()
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    block_radius_cells = max(
        0, math.ceil((float(route_width_um) / 2.0) / float(grid.grid_size_um))
    )
    router.set_static_cells(sorted(obstacle_map.blocked_cells))
    router.clear_port_open_cells()
    router.add_port_open_cells(sorted(port_open_cells))
    net_id = 0

    t_astar_start = 0.0
    if debug_timing:
        t_astar_start = time.perf_counter()

    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")

            abs_port1 = get_port_from_instance(routed_layout, inst1, port1)
            abs_port2 = get_port_from_instance(routed_layout, inst2, port2)

            source_access = router.build_port_access(
                f"{inst1},{port1}",
                float(abs_port1.center[0]),
                float(abs_port1.center[1]),
                None if abs_port1.orientation is None else float(abs_port1.orientation),
            )
            target_access = router.build_port_access(
                f"{inst2},{port2}",
                float(abs_port2.center[0]),
                float(abs_port2.center[1]),
                _flip_orientation(abs_port2.orientation),
            )

            source = (
                int(source_access.anchor_cell[0]),
                int(source_access.anchor_cell[1]),
                int(source_access.entry_angle),
            )
            target = (
                int(target_access.anchor_cell[0]),
                int(target_access.anchor_cell[1]),
                int(target_access.entry_angle),
            )

            print(f"  Routing {net_name}: {port1_spec} -> {port2_spec}...", end=" ")

            net_id += 1
            route_obj = router.route_single_net_and_commit(
                net_id,
                rust_backend.State(*source),
                rust_backend.State(*target),
                block_radius_cells,
            )

            polygon = router.realize_route_polygon_with_port_access(
                route_obj,
                float(route_width_um),
                source_access,
                target_access,
            )
            routed_layout.add_polygon(polygon, layer=route_layer)

            if debug_path is not None:
                route_dir = debug_path / "routes"
                _ensure_dir(route_dir)
                route_svg = route_dir / f"{debug_prefix}_{net_name}.svg"
                route_svg.write_text(router.export_debug_svg(route_obj), encoding="utf-8")
                route_svgs.append(route_svg)

            print("ok")

    if debug_timing:
        t_astar_end = time.perf_counter()
        print(f"      - Astar time: {t_astar_end - t_astar_start:.4f} s")

    return routed_layout, RustRouteDebugArtifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
    )


def _port_to_state(port: Port, grid: GridSpec, *, is_target: bool) -> tuple[int, int, int]:
    gx, gy = physical_to_grid(port.center[0], port.center[1], grid)
    angle = _orientation_to_angle(port.orientation, flip=is_target)
    return gx, gy, angle
