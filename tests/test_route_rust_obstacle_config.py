import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from photonic_router.routing_layers import get_routing_obstacle_layers
from translation.route_rust import _resolve_obstacle_config


def test_resolve_obstacle_config_defaults_to_route_layer_when_missing():
    resolved = _resolve_obstacle_config(None, route_layer=(1, 0))

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.obstacle_layers == ((1, 0),)
    assert resolved.obstacle_mode == "bounding_boxes"
    assert resolved.clear_port_open_cells_from_static is False


def test_resolve_obstacle_config_can_include_heater_layers():
    resolved = _resolve_obstacle_config(
        None,
        route_layer=(1, 0),
        include_heater_obstacles=True,
    )

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.obstacle_layers == get_routing_obstacle_layers(include_heaters=True)


def test_resolve_obstacle_config_keeps_optical_only_when_heaters_disabled():
    resolved = _resolve_obstacle_config(
        None,
        route_layer=(1, 0),
        include_heater_obstacles=False,
    )

    assert isinstance(resolved, StaticObstacleMapConfig)
    assert resolved.obstacle_layers == get_routing_obstacle_layers(include_heaters=False)


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


def test_dense_obstacle_cell_cap_covers_the_whole_map_and_keeps_the_default_for_small_maps():
    """The lidar-mode crossing hook rasterizes the full routing bounds; the
    kernel cap must cover the map (benes_64x64: 8380 x 3439 cells = 28.8M >
    the 10M default, which panicked on 2026-09-13)."""
    from translation.route_rust import DENSE_OBSTACLE_CELL_CAP_MARGIN, dense_obstacle_cell_cap

    assert dense_obstacle_cell_cap(1000, 500, 10_000_000) == 10_000_000
    assert dense_obstacle_cell_cap(8380, 3439, 10_000_000) == DENSE_OBSTACLE_CELL_CAP_MARGIN * 8380 * 3439
    assert dense_obstacle_cell_cap(8380, 3439, 10_000_000) > 28_800_000
