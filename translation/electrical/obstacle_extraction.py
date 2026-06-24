"""Build a layer-filtered obstacle map for electrical routing."""

from __future__ import annotations

from gdsfactory.component import Component

from photonic_router.benchmark_extractor import BBox, extract_benchmark
from photonic_router.static_obstacle_builder import (
    StaticObstacleMapConfig,
    build_static_obstacle_map,
)

from .pitch_grid import bbox_to_grid_cells, disk_cells
from .port_access import build_electrical_port_accesses
from .terminal_contacts import terminal_contact_seed_point_for_side
from .types import (
    BusStripe,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    TerminalPairGroup,
)


def build_electrical_obstacle_map(
    component: Component,
    terminal_groups: tuple[TerminalPairGroup, ...],
    config: ElectricalRoutingConfig,
) -> ElectricalObstacleMap:
    """Build electrical obstacles from configured metal/heater layers only."""

    config.validate()
    all_geometry = extract_benchmark(component)
    die_bbox = _electrical_die_bbox(
        all_geometry.bbox,
        config,
        individual_route_count=len(terminal_groups),
    )
    static_config = StaticObstacleMapConfig(
        grid_size_um=config.routing_grid_pitch_um,
        security_margin_um=0.0,
        clearance_um=config.obstacle_clearance_um,
        heater_clearance_um=None,
        clearance_metric=config.clearance_metric,
        port_open_radius_um=0.0,
        obstacle_mode=config.obstacle_mode,
        clear_port_open_cells_from_static=False,
        materialize_bbox_cells=True,
        populate_obstacle_map=True,
        die_bbox=die_bbox,
        obstacle_layers=config.metal_obstacle_layers,
        heater_obstacle_layers=None,
    )
    static_obstacles = build_static_obstacle_map(component, config=static_config)
    grid = static_obstacles.grid

    bus_bbox = _bus_bbox(all_geometry.bbox, config)
    bus_cells = bbox_to_grid_cells(bus_bbox, grid)

    common_bus_terminal_open_cells: dict[str, frozenset[tuple[int, int]]] = {}
    individual_terminal_open_cells: dict[str, frozenset[tuple[int, int]]] = {}
    terminal_open_cells: dict[str, frozenset[tuple[int, int]]] = {}
    for terminal in _iter_terminals(terminal_groups):
        common_bus_cells = disk_cells(
            terminal_contact_seed_point_for_side(terminal, config.bus_side),
            config.terminal_open_radius_um,
            grid,
        )
        individual_cells = disk_cells(
            terminal_contact_seed_point_for_side(terminal, config.pad_side),
            config.terminal_open_radius_um,
            grid,
        )
        common_bus_terminal_open_cells[terminal.id] = frozenset(common_bus_cells)
        individual_terminal_open_cells[terminal.id] = frozenset(individual_cells)
        terminal_open_cells[terminal.id] = frozenset(
            set(common_bus_cells) | set(individual_cells)
        )

    cleared_cells = set(bus_cells)
    for cells in terminal_open_cells.values():
        cleared_cells.update(cells)

    blocked = set(static_obstacles.blocked_cells)
    blocked.difference_update(cleared_cells)
    common_bus_port_accesses, individual_port_accesses = build_electrical_port_accesses(
        terminal_groups,
        common_bus_open_cells=common_bus_terminal_open_cells,
        individual_open_cells=individual_terminal_open_cells,
        blocked_cells=frozenset(blocked),
        grid=grid,
        config=config,
    )

    return ElectricalObstacleMap(
        grid=grid,
        raw_blocked_cells=frozenset(static_obstacles.raw_blocked_cells),
        blocked_cells=frozenset(blocked),
        terminal_open_cells=terminal_open_cells,
        bus=BusStripe(
            side=config.bus_side,
            bbox=bus_bbox,
            cells=frozenset(bus_cells),
        ),
        die_bbox=die_bbox,
        layout_bbox=all_geometry.bbox,
        raw_obstacle_bboxes=tuple(
            dict.fromkeys(
                _polygon_bbox(polygon)
                for polygon in static_obstacles.benchmark.polygons
            )
        ),
        common_bus_terminal_open_cells=common_bus_terminal_open_cells,
        individual_terminal_open_cells=individual_terminal_open_cells,
        common_bus_port_accesses=common_bus_port_accesses,
        individual_port_accesses=individual_port_accesses,
    )


def _iter_terminals(groups: tuple[TerminalPairGroup, ...]) -> tuple[ElectricalTerminal, ...]:
    return tuple(terminal for group in groups for terminal in group.terminals)


def _polygon_bbox(polygon: list[tuple[float, float]]) -> BBox:
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    return (min(xs), min(ys), max(xs), max(ys))


def _electrical_die_bbox(
    layout_bbox: BBox,
    config: ElectricalRoutingConfig,
    *,
    individual_route_count: int,
) -> BBox:
    xmin, ymin, xmax, ymax = layout_bbox
    margin = config.layout_margin_um
    x_margin = max(
        margin,
        config.bus_x_margin_um + config.pad_pitch_um + config.bondpad_width_um,
    )
    bus_extra = config.bus_offset_um + config.bus_width_um + margin
    pad_offset_um = max(
        config.pad_offset_um,
        _required_pad_channel_offset_um(individual_route_count, config),
    )
    pad_extra = pad_offset_um + config.bondpad_length_um + margin
    top_extra = pad_extra if config.pad_side == "top" else bus_extra
    bottom_extra = pad_extra if config.pad_side == "bottom" else bus_extra
    if config.bus_side == "bottom":
        return (
            xmin - x_margin,
            ymin - bottom_extra,
            xmax + x_margin,
            ymax + top_extra,
        )
    return (
        xmin - x_margin,
        ymin - bottom_extra,
        xmax + x_margin,
        ymax + top_extra,
    )


def _required_pad_channel_offset_um(
    individual_route_count: int,
    config: ElectricalRoutingConfig,
) -> float:
    if individual_route_count <= 0:
        return config.pad_offset_um
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    return individual_route_count * track_pitch_um + config.wire_width_um


def _bus_bbox(layout_bbox: BBox, config: ElectricalRoutingConfig) -> BBox:
    xmin, ymin, xmax, ymax = layout_bbox
    x0 = xmin - config.bus_x_margin_um
    x1 = xmax + config.bus_x_margin_um
    pad_side_extension = config.pad_pitch_um + config.bondpad_width_um
    if config.common_bus_pad_position == "left":
        x0 -= pad_side_extension
    else:
        x1 += pad_side_extension
    if config.bus_side == "bottom":
        y1 = ymin - config.bus_offset_um
        y0 = y1 - config.bus_width_um
    else:
        y0 = ymax + config.bus_offset_um
        y1 = y0 + config.bus_width_um
    return (x0, y0, x1, y1)
