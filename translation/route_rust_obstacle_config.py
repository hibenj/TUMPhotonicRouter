"""Obstacle layer configuration resolution for the Rust-backed photonic router."""

from __future__ import annotations

import importlib
from dataclasses import is_dataclass, replace
from typing import Any, cast

from photonic_router.routing_layers import get_routing_obstacle_layers

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig


def _grid_origin_xy(grid: GridSpec) -> tuple[float, float]:
    if hasattr(grid, "origin"):
        origin = getattr(grid, "origin")
        return float(origin[0]), float(origin[1])
    return (
        float(getattr(grid, "origin_x_um", 0.0)),
        float(getattr(grid, "origin_y_um", 0.0)),
    )


def _default_obstacle_layers(
    route_layer: tuple[int, int],
    *,
    include_heater_obstacles: bool = False,
) -> tuple[tuple[int, int], ...]:
    if route_layer == (1, 0):
        return get_routing_obstacle_layers(include_heaters=include_heater_obstacles)
    return ((int(route_layer[0]), int(route_layer[1])),)


def _resolve_obstacle_config(
    obstacle_config: object | None,
    *,
    route_layer: tuple[int, int],
    include_heater_obstacles: bool = False,
) -> object:
    """Default obstacle extraction to the photonic routing layer."""

    default_layers = _default_obstacle_layers(
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


def _with_bbox_cell_materialization(
    obstacle_config: object | None,
    *,
    materialize_bbox_cells: bool,
    populate_obstacle_map: bool = True,
) -> object | None:
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


def _with_obstacle_mode(
    obstacle_config: object | None,
    *,
    obstacle_mode: str,
    clear_port_open_cells_from_static: bool | None = None,
    populate_obstacle_map: bool | None = None,
    materialize_cell_sets: bool | None = None,
) -> object | None:
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


def _schematic_instance_component_name(
    schematic: object,
    instance_name: str,
) -> str | None:
    netlist = getattr(schematic, "netlist", None)
    instances = getattr(netlist, "instances", None)
    if not isinstance(instances, dict):
        return None
    instance = instances.get(instance_name)
    if instance is None:
        return None
    return _coerce_component_name(instance)


def _port_type_name(port: object) -> str | None:
    port_type = getattr(port, "port_type", None)
    if port_type is None:
        return None
    return str(port_type)
