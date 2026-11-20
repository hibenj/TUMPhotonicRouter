"""Obstacle layer configuration resolution for the Rust-backed photonic router."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any, TypeAlias

from photonic_router.routing_layers import get_routing_obstacle_layers
from photonic_router.static_obstacle_builder import GridSpec, StaticObstacleMapConfig


def grid_origin_xy(grid: GridSpec) -> tuple[float, float]:
    """Return the origin (x, y) of a grid spec in um.

    Reads ``grid.origin`` if present, otherwise ``origin_x_um`` and ``origin_y_um``,
    each 0.0 if missing.
    """
    if hasattr(grid, "origin"):
        origin = grid.origin
        return float(origin[0]), float(origin[1])
    return (
        float(getattr(grid, "origin_x_um", 0.0)),
        float(getattr(grid, "origin_y_um", 0.0)),
    )


def default_obstacle_layers(
    route_layer: tuple[int, int],
    *,
    include_heater_obstacles: bool = False,
) -> tuple[tuple[int, int], ...]:
    """Return the layers whose shapes are obstacles for routes on ``route_layer``.

    For the layer (1, 0) these are the layers of `get_routing_obstacle_layers`, with
    the heater layers if asked for. For any other route layer it is that layer alone.
    """
    if route_layer == (1, 0):
        return get_routing_obstacle_layers(include_heaters=include_heater_obstacles)
    return ((int(route_layer[0]), int(route_layer[1])),)


#: What a caller may hand in as an obstacle configuration: the record, the
#: keywords of `StaticObstacleMapConfig` as a mapping, or None for the defaults.
ObstacleConfigInput: TypeAlias = "StaticObstacleMapConfig | Mapping[str, Any] | None"


def _as_obstacle_config(obstacle_config: ObstacleConfigInput) -> StaticObstacleMapConfig:
    """Return the configuration record for any accepted form; refuse anything else."""
    if obstacle_config is None:
        return StaticObstacleMapConfig()
    if isinstance(obstacle_config, StaticObstacleMapConfig):
        return obstacle_config
    if isinstance(obstacle_config, Mapping):
        return StaticObstacleMapConfig(**obstacle_config)
    raise TypeError(
        "obstacle_config must be a StaticObstacleMapConfig, a mapping of its fields or None, "
        f"not {type(obstacle_config).__name__}"
    )


def resolve_obstacle_config(
    obstacle_config: ObstacleConfigInput,
    *,
    route_layer: tuple[int, int],
    include_heater_obstacles: bool = False,
) -> StaticObstacleMapConfig:
    """Return the configuration with its obstacle layers defaulted to the routing layer.

    Layers the caller named are kept; only a missing ``obstacle_layers`` is
    filled with `default_obstacle_layers`.
    """
    config = _as_obstacle_config(obstacle_config)
    if config.obstacle_layers is not None:
        return config
    return replace(
        config,
        obstacle_layers=default_obstacle_layers(
            route_layer, include_heater_obstacles=include_heater_obstacles
        ),
    )


def with_bbox_cell_materialization(
    obstacle_config: ObstacleConfigInput,
    *,
    materialize_bbox_cells: bool,
    populate_obstacle_map: bool = True,
) -> StaticObstacleMapConfig:
    """Return the obstacle config with ``materialize_bbox_cells`` and ``populate_obstacle_map`` set."""
    return replace(
        _as_obstacle_config(obstacle_config),
        materialize_bbox_cells=materialize_bbox_cells,
        populate_obstacle_map=populate_obstacle_map,
    )


def with_obstacle_mode(
    obstacle_config: ObstacleConfigInput,
    *,
    obstacle_mode: str,
    clear_port_open_cells_from_static: bool | None = None,
    populate_obstacle_map: bool | None = None,
    materialize_cell_sets: bool | None = None,
) -> StaticObstacleMapConfig:
    """Return the obstacle config with ``obstacle_mode`` set, plus the optional flags given.

    A flag left at None keeps the value the configuration has.
    """
    config = replace(_as_obstacle_config(obstacle_config), obstacle_mode=obstacle_mode)
    if clear_port_open_cells_from_static is not None:
        config = replace(
            config, clear_port_open_cells_from_static=clear_port_open_cells_from_static
        )
    if populate_obstacle_map is not None:
        config = replace(config, populate_obstacle_map=populate_obstacle_map)
    if materialize_cell_sets is not None:
        config = replace(config, materialize_cell_sets=materialize_cell_sets)
    return config


def _coerce_component_name(instance: object) -> str | None:
    component_obj = getattr(instance, "component", None)
    if component_obj is None:
        return None
    if isinstance(component_obj, str):
        return component_obj
    name = getattr(component_obj, "name", None)
    if isinstance(name, str) and name:
        return name
    return None


def schematic_instance_component_name(
    schematic: object,
    instance_name: str,
) -> str | None:
    """Return the component name of an instance of a schematic, or None.

    None if the schematic's netlist has no instance dict, if it has no instance of
    that name, or if no name can be read from the instance.
    """
    netlist = getattr(schematic, "netlist", None)
    instances = getattr(netlist, "instances", None)
    if not isinstance(instances, dict):
        return None
    instance = instances.get(instance_name)
    if instance is None:
        return None
    return _coerce_component_name(instance)


def port_type_name(port: object) -> str | None:
    """Return the ``port_type`` of a port as a string, or None if the port has none."""
    port_type = getattr(port, "port_type", None)
    if port_type is None:
        return None
    return str(port_type)
