"""Phase 1: the static obstacle context -- obstacle map, crossing device info, debug SVG path."""

from __future__ import annotations

from pathlib import Path

from photonic_router.static_obstacle_builder import build_static_obstacle_map

from translation.route_rust_crossing_plan import _resolve_crossing_half_size_cells
from translation.route_rust_debug_artifacts import _ensure_dir
from translation.route_rust_geometry import _format_route_indices
from translation.route_rust_obstacle_config import _resolve_obstacle_config
from translation.route_rust_types import _as_float

from translation.routing import timing
from translation.routing.settings import SessionSettings
from translation.routing.stages import ObstacleContext
from translation.routing.state import SessionState

def build_static_obstacle_context(
    settings: SessionSettings, state: SessionState
) -> ObstacleContext:
    """Build the static obstacle map, resolve grid/crossing sizing, export debug artifacts if enabled, and validate the Rust backend.

    Sets `self.grid`, `self.resolved_crossing_half_size_cells`, `self.debug_path`,
    `self.diagnostics_enabled`, and `self.route_svgs`. Prints the "Routing N nets..."
    header. Returns the obstacle map, the crossing-device info dict
    `_resolve_crossing_half_size_cells` produces (consumed later when building
    `self.crossing_plan_info`), and the debug obstacle SVG path (or `None` if debug
    output is disabled).
    """
    t_obstacle_start = timing.pipeline_timer_start(settings, state)
    state.resolved_obstacle_config = _resolve_obstacle_config(
        settings.obstacle_config,
        route_layer=settings.route_layer,
        include_heater_obstacles=settings.include_heater_obstacles,
    )
    obstacle_map = build_static_obstacle_map(
        settings.unrouted_layout, config=state.resolved_obstacle_config
    )
    timing.record_pipeline_timing(settings, state, "obstacle_map", t_obstacle_start)
    if settings.debug_timing and settings.verbose_route_diagnostics:
        print(
            "      - Obstacle Map time: "
            f"{state.route_nets_timings_s.get('obstacle_map', 0.0):.4f} s"
        )
    state.grid = obstacle_map.grid
    state.resolved_crossing_half_size_cells, crossing_device_info = (
        _resolve_crossing_half_size_cells(
            requested_half_size_cells=int(settings.crossing_half_size_cells),
            enable_crossings=bool(settings.enable_crossings),
            grid_size_um=float(state.grid.grid_size_um),
            clearance_um=_as_float(
                getattr(state.resolved_obstacle_config, "clearance_um", 0.0),
                0.0,
            ),
        )
    )

    state.debug_path = (
        Path(settings.debug_dir) if settings.debug_dir is not None else None
    )
    state.diagnostics_enabled = state.debug_path is not None
    obstacle_svg = None
    state.route_svgs: list[Path] = []

    if state.debug_path is not None:
        obstacle_dir = state.debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{settings.debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)
        route_dir = state.debug_path / "routes"
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

    if not hasattr(state.rust_backend, "PyPhotonicRouter"):
        raise RuntimeError(
            "Rust backend does not expose PyPhotonicRouter. "
            "Rebuild/install the Rust extension with the class-based API."
        )

    return ObstacleContext(obstacle_map, crossing_device_info, obstacle_svg)
