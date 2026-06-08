import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from translation.route_rust import _resolve_obstacle_config


def test_resolve_obstacle_config_defaults_to_route_layer_when_missing():
    resolved = _resolve_obstacle_config(None, route_layer=(1, 0))

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.obstacle_layers == ((1, 0),)
    assert resolved.obstacle_mode == "bounding_boxes"
    assert resolved.clear_port_open_cells_from_static is False


def test_resolve_obstacle_config_preserves_existing_layers():
    config = StaticObstacleMapConfig(
        grid_size_um=0.25,
        obstacle_layers=((1, 0), (2, 0)),
    )

    resolved = _resolve_obstacle_config(config, route_layer=(1, 0))

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.grid_size_um == 0.25
    assert resolved.obstacle_layers == ((1, 0), (2, 0))


def test_resolve_obstacle_config_sets_default_layers_for_partial_dataclass():
    config = StaticObstacleMapConfig(grid_size_um=0.25, obstacle_layers=None)

    resolved = _resolve_obstacle_config(config, route_layer=(3, 1))

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.grid_size_um == 0.25
    assert resolved.obstacle_layers == ((3, 1),)


def test_resolve_obstacle_config_sets_default_layers_for_dict():
    resolved = _resolve_obstacle_config(
        {"grid_size_um": 1.0, "obstacle_layers": None},
        route_layer=(7, 2),
    )

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.grid_size_um == 1.0
    assert resolved.obstacle_layers == ((7, 2),)
    assert resolved.clear_port_open_cells_from_static is False


def test_resolve_obstacle_config_preserves_obstacle_mode():
    config = StaticObstacleMapConfig(
        obstacle_mode="bounding_boxes",
    )

    resolved = _resolve_obstacle_config(config, route_layer=(1, 0))

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.obstacle_mode == "bounding_boxes"


def test_resolve_obstacle_config_preserves_clear_port_opening_option():
    config = StaticObstacleMapConfig(
        clear_port_open_cells_from_static=False,
        obstacle_mode="rasterized_polygons",
    )

    resolved = _resolve_obstacle_config(config, route_layer=(1, 0))

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.clear_port_open_cells_from_static is False
