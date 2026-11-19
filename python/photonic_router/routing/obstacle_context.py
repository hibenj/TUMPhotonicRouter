"""Phase 1: the static obstacle context -- obstacle map, crossing device info, debug SVG path."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from photonic_router.crossings.route_rust_crossing_plan import _resolve_crossing_half_size_cells
from photonic_router.gds.route_rust_debug_artifacts import _ensure_dir
from photonic_router.records.route_rust_geometry import _format_route_indices
from photonic_router.records.route_rust_obstacle_config import _resolve_obstacle_config
from photonic_router.records.route_rust_types import _as_float
from photonic_router.routing import timing
from photonic_router.routing.products import ObstacleContext
from photonic_router.routing.settings import SessionSettings
from photonic_router.static_obstacle_builder import build_static_obstacle_map


def build_static_obstacle_context(
    settings: SessionSettings, rust_backend: Any, timings_s: dict[str, float]
) -> ObstacleContext:
    """Phase 1: build the static obstacle map and resolve what depends on it.

    Resolves the obstacle configuration, builds the map, resolves the
    crossing footprint against the grid, writes the obstacle SVG and clears
    the route artifacts of an earlier run when a debug directory is set, and
    prints the "Routing N nets..." header.

    Args:
        settings: the session's resolved inputs.
        rust_backend: the loaded kernel module; refused when it lacks the
            class-based router.
        timings_s: the pipeline timing buckets; "obstacle_map" is added.

    Raises:
        RuntimeError: the kernel module does not expose `PyPhotonicRouter`.
    """
    t_obstacle_start = timing.pipeline_timer_start(settings)
    resolved_obstacle_config = _resolve_obstacle_config(
        settings.obstacle_config,
        route_layer=settings.route_layer,
        include_heater_obstacles=settings.include_heater_obstacles,
    )
    obstacle_map = build_static_obstacle_map(
        settings.unrouted_layout, config=resolved_obstacle_config
    )
    timing.record_elapsed(settings, timings_s, "obstacle_map", t_obstacle_start)
    if settings.debug_timing and settings.verbose_route_diagnostics:
        print(f"      - Obstacle Map time: {timings_s.get('obstacle_map', 0.0):.4f} s")
    grid = obstacle_map.grid
    resolved_crossing_half_size_cells, crossing_device_info = _resolve_crossing_half_size_cells(
        requested_half_size_cells=int(settings.crossing_half_size_cells),
        enable_crossings=bool(settings.enable_crossings),
        grid_size_um=float(grid.grid_size_um),
        clearance_um=_as_float(
            getattr(resolved_obstacle_config, "clearance_um", 0.0),
            0.0,
        ),
    )

    debug_path = Path(settings.debug_dir) if settings.debug_dir is not None else None
    obstacle_svg = None

    if debug_path is not None:
        obstacle_dir = debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{settings.debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)
        route_dir = debug_path / "routes"
        if route_dir.exists():
            for old_artifact in route_dir.glob(f"{settings.debug_prefix}_*"):
                if old_artifact.is_file() and old_artifact.suffix.lower() in {".svg", ".txt"}:
                    old_artifact.unlink()

    nets = settings.schematic.netlist.routes
    if settings.debug_route_indices is None:
        print(f"\nRouting {len(nets)} nets using Rust router...")
    else:
        selected = _format_route_indices(settings.debug_route_indices)
        print(
            f"\nRouting {len(nets)} nets using Rust router "
            f"(printing/exporting route SVGs for indices: {selected})..."
        )

    if not hasattr(rust_backend, "PyPhotonicRouter"):
        raise RuntimeError(
            "Rust backend does not expose PyPhotonicRouter. "
            "Rebuild/install the Rust extension with the class-based API."
        )

    return ObstacleContext(
        obstacle_map=obstacle_map,
        crossing_device_info=crossing_device_info,
        obstacle_svg=obstacle_svg,
        grid=grid,
        resolved_obstacle_config=resolved_obstacle_config,
        resolved_crossing_half_size_cells=resolved_crossing_half_size_cells,
        debug_path=debug_path,
    )
