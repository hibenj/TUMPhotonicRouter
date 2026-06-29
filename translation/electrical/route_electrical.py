"""Orchestrate the first electrical heater-routing milestones."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from .bundle_detail_router import route_detailed_bundles
from .common_bus_router import route_common_bus
from .debug import export_electrical_debug_svg, export_electrical_metal_snapshot_svg
from .escape_router import route_common_bus_escape
from .individual_topology import compute_individual_escape_topology
from .metal_realization import realize_electrical_metal
from .obstacle_extraction import build_electrical_obstacle_map
from .pad_slots import plan_pad_slots
from .pitch_grid import bbox_to_grid_cells
from .terminal_extraction import extract_heater_terminal_pairs
from .types import (
    BusStripe,
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
    GridCell,
)
from .verification import verify_electrical_routing


def route_electrical_heaters(
    component: Component,
    schematic: Schematic | None = None,
    config: ElectricalRoutingConfig | None = None,
    *,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "electrical",
) -> ElectricalRoutingResult:
    """Run the current electrical heater-routing milestones.

    It extracts heater terminal pairs, builds the electrical obstacle grid,
    routes one terminal per heater to the derived opposite-side common bus,
    plans abstract pad slots, routes the remaining individual terminals to
    those slots, realizes metal polygons, and optionally writes a debug SVG.
    """

    config = config or ElectricalRoutingConfig()
    config.validate()

    terminal_groups = extract_heater_terminal_pairs(component, schematic, config)
    obstacle_map = build_electrical_obstacle_map(component, terminal_groups, config)
    if not terminal_groups:
        routed_component = component.copy()
        return ElectricalRoutingResult(
            terminal_groups=terminal_groups,
            obstacle_map=obstacle_map,
            common_bus=CommonBusRoutingResult(
                bus_side=config.bus_side,
                bus=obstacle_map.bus,
                selected_terminals={},
                unselected_terminals={},
                routes=(),
                tree_cells=frozenset(),
                failed_heaters=(),
            ),
            routed_component=routed_component,
        )

    common_bus = route_common_bus(terminal_groups, obstacle_map, config)
    obstacle_map, common_bus = _trim_common_bus_to_connections(
        obstacle_map,
        common_bus,
        config,
    )
    individual_topology = (
        compute_individual_escape_topology(obstacle_map, common_bus, config)
        if common_bus.success
        else None
    )
    pad_plan = (
        plan_pad_slots(common_bus, obstacle_map, config, individual_topology)
        if common_bus.success
        else None
    )
    common_bus_escape = (
        route_common_bus_escape(obstacle_map, common_bus, pad_plan, config)
        if pad_plan is not None
        else None
    )
    detailed_bundle_routes = (
        route_detailed_bundles(
            obstacle_map,
            common_bus,
            common_bus_escape,
            individual_topology,
            pad_plan,
            config,
        )
        if (
            pad_plan is not None
            and common_bus_escape is not None
            and individual_topology is not None
            and common_bus_escape.success
        )
        else None
    )
    routed_component = (
        realize_electrical_metal(
            component,
            obstacle_map,
            common_bus,
            common_bus_escape,
            detailed_bundle_routes,
            pad_plan,
            config,
        )
        if common_bus.success
        else None
    )
    verification = (
        verify_electrical_routing(
            obstacle_map,
            common_bus,
            common_bus_escape,
            detailed_bundle_routes,
            pad_plan,
            config,
        )
        if routed_component is not None
        else None
    )
    artifacts: dict[str, str] = {}
    if debug_dir is not None:
        debug_path = Path(debug_dir) / "electrical" / f"{debug_prefix}_common_bus.svg"
        export_electrical_debug_svg(
            debug_path,
            obstacle_map,
            terminal_groups,
            common_bus,
            common_bus_escape=common_bus_escape,
            individual_topology=individual_topology,
            detailed_bundle_routes=detailed_bundle_routes,
            pad_plan=pad_plan,
        )
        artifacts["common_bus_svg"] = str(debug_path)
        if routed_component is not None:
            metal_snapshot_path = (
                Path(debug_dir) / "electrical" / f"{debug_prefix}_metal_snapshot.svg"
            )
            export_electrical_metal_snapshot_svg(
                metal_snapshot_path,
                routed_component,
                obstacle_map,
                terminal_groups,
                pad_plan,
                config,
            )
            artifacts["metal_snapshot_svg"] = str(metal_snapshot_path)

    return ElectricalRoutingResult(
        terminal_groups=terminal_groups,
        obstacle_map=obstacle_map,
        common_bus=common_bus,
        pad_plan=pad_plan,
        common_bus_escape=common_bus_escape,
        individual_topology=individual_topology,
        detailed_bundle_routes=detailed_bundle_routes,
        routed_component=routed_component,
        verification=verification,
        debug_artifacts=artifacts,
    )


def _trim_common_bus_to_connections(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    config: ElectricalRoutingConfig,
) -> tuple[ElectricalObstacleMap, CommonBusRoutingResult]:
    """Limit realized/debug bus stripe to the span touched by routed terminals."""

    if not common_bus.routes:
        return obstacle_map, common_bus

    provisional_bus_cells = obstacle_map.bus.cells
    connection_cells = {
        cell
        for route in common_bus.routes
        for cell in route.path
        if cell in provisional_bus_cells
    }
    if not connection_cells:
        return obstacle_map, common_bus

    min_x_um = min(
        _grid_cell_center_um(cell, obstacle_map)[0]
        for cell in connection_cells
    )
    max_x_um = max(
        _grid_cell_center_um(cell, obstacle_map)[0]
        for cell in connection_cells
    )
    half_overlap_um = max(
        obstacle_map.grid.grid_size_um / 2.0,
        config.wire_width_um / 2.0,
    )
    _, bus_ymin, _, bus_ymax = obstacle_map.bus.bbox
    trimmed_bbox = (
        min_x_um - half_overlap_um,
        bus_ymin,
        max_x_um + half_overlap_um,
        bus_ymax,
    )
    trimmed_bus_cells = bbox_to_grid_cells(trimmed_bbox, obstacle_map.grid)
    trimmed_bus = BusStripe(
        side=obstacle_map.bus.side,
        bbox=trimmed_bbox,
        cells=frozenset(trimmed_bus_cells),
    )
    trimmed_tree_cells = (
        set(common_bus.tree_cells).difference(provisional_bus_cells)
        | set(trimmed_bus_cells)
    )
    trimmed_obstacle_map = replace(obstacle_map, bus=trimmed_bus)
    trimmed_common_bus = replace(
        common_bus,
        bus=trimmed_bus,
        tree_cells=frozenset(trimmed_tree_cells),
    )
    return trimmed_obstacle_map, trimmed_common_bus


def _grid_cell_center_um(
    cell: GridCell,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[float, float]:
    x, y = cell
    grid = obstacle_map.grid
    origin_x, origin_y = grid.origin
    return (
        origin_x + (x + 0.5) * grid.grid_size_um,
        origin_y + (y + 0.5) * grid.grid_size_um,
    )
