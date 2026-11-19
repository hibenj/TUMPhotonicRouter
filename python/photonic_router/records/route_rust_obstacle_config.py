"""Obstacle layer configuration resolution for the Rust-backed photonic router."""

from __future__ import annotations

import importlib
from dataclasses import is_dataclass, replace
from typing import Any, cast

from photonic_router.routing_layers import get_routing_obstacle_layers

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig


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


def resolve_obstacle_config(
    obstacle_config: object | None,
    *,
    route_layer: tuple[int, int],
    include_heater_obstacles: bool = False,
) -> object:
    """Default obstacle extraction to the photonic routing layer."""

    default_layers = default_obstacle_layers(
        route_layer,
        include_heater_obstacles=include_heater_obstacles,
    )
    if obstacle_config is None:
        return StaticObstacleMapConfig(obstacle_layers=default_layers)

    if isinstance(obstacle_config, dict):
        if obstacle_config.get("obstacle_layers") is None:
            config_dict = dict(obstacle_config)
            config_dict["obstacle_layers"] = default_layers
            return StaticObstacleMapConfig(**config_dict)
        return StaticObstacleMapConfig(**obstacle_config)

    obstacle_layers = getattr(obstacle_config, "obstacle_layers", None)
    if obstacle_layers is None:
        if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
            try:
                return replace(cast(Any, obstacle_config), obstacle_layers=default_layers)
            except Exception:
                return obstacle_config
        return obstacle_config

    return obstacle_config


def with_bbox_cell_materialization(
    obstacle_config: object | None,
    *,
    materialize_bbox_cells: bool,
    populate_obstacle_map: bool = True,
) -> object | None:
    """Return the obstacle config with ``materialize_bbox_cells`` and ``populate_obstacle_map`` set.

    None gives a new dict with the two keys, a dict a changed copy, a dataclass
    instance a changed copy made with ``dataclasses.replace``. A dataclass that
    rejects the fields and any other object are returned unchanged.
    """
    if obstacle_config is None:
        return {
            "materialize_bbox_cells": materialize_bbox_cells,
            "populate_obstacle_map": populate_obstacle_map,
        }

    if isinstance(obstacle_config, dict):
        config_dict = dict(obstacle_config)
        config_dict["materialize_bbox_cells"] = materialize_bbox_cells
        config_dict["populate_obstacle_map"] = populate_obstacle_map
        return config_dict

    if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
        try:
            return replace(
                cast(Any, obstacle_config),
                materialize_bbox_cells=materialize_bbox_cells,
                populate_obstacle_map=populate_obstacle_map,
            )
        except Exception:
            return obstacle_config

    return obstacle_config


def with_obstacle_mode(
    obstacle_config: object | None,
    *,
    obstacle_mode: str,
    clear_port_open_cells_from_static: bool | None = None,
    populate_obstacle_map: bool | None = None,
    materialize_cell_sets: bool | None = None,
) -> object | None:
    """Return the obstacle config with ``obstacle_mode`` set, plus the optional flags given.

    A flag left at None is not written. None gives a new dict with the updates, a dict
    a changed copy, a dataclass instance a changed copy made with
    ``dataclasses.replace``. A dataclass that rejects the fields and any other object
    are returned unchanged.
    """
    updates: dict[str, object] = {"obstacle_mode": obstacle_mode}
    if clear_port_open_cells_from_static is not None:
        updates["clear_port_open_cells_from_static"] = clear_port_open_cells_from_static
    if populate_obstacle_map is not None:
        updates["populate_obstacle_map"] = populate_obstacle_map
    if materialize_cell_sets is not None:
        updates["materialize_cell_sets"] = materialize_cell_sets

    if obstacle_config is None:
        return updates

    if isinstance(obstacle_config, dict):
        config_dict = dict(obstacle_config)
        config_dict.update(updates)
        return config_dict

    if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
        try:
            return replace(cast(Any, obstacle_config), **updates)
        except Exception:
            return obstacle_config

    return obstacle_config


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
