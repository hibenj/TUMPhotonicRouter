import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from photonic_router.static_obstacle_builder import _load_rust_backend


def test_rust_backend_exposes_router_class():
    backend = _load_rust_backend()
    assert backend is not None
    assert hasattr(backend, "PyPhotonicRouter")
    assert hasattr(backend, "GridSpec")
    assert hasattr(backend, "CrossingConfig")
    assert hasattr(backend, "CrossingConstraint")
    assert hasattr(backend, "build_static_obstacle_map_rs")


def test_astar_config_exposes_jps4_flag():
    backend = _load_rust_backend()
    assert backend is not None
    cfg = backend.AStarConfig()

    assert cfg.enable_jps4 is False
    cfg.enable_jps4 = True
    assert cfg.enable_jps4 is True


def test_astar_config_exposes_proactive_congestion_fields():
    backend = _load_rust_backend()
    assert backend is not None
    cfg = backend.AStarConfig()

    assert cfg.proactive_congestion_weight == 0.0
    assert cfg.proactive_congestion_radius_cells == 0
    cfg.proactive_congestion_weight = 2.5
    cfg.proactive_congestion_radius_cells = 4
    assert cfg.proactive_congestion_weight == 2.5
    assert cfg.proactive_congestion_radius_cells == 4


def test_primitive_config_exposes_grid_experiment_flags():
    backend = _load_rust_backend()
    assert backend is not None
    cfg = backend.PrimitiveLibraryConfig()

    assert cfg.jps4_unit_grid is False
    assert cfg.grid4_unit_grid is False
    cfg.jps4_unit_grid = True
    cfg.grid4_unit_grid = True
    assert cfg.jps4_unit_grid is True
    assert cfg.grid4_unit_grid is True


def test_router_crossing_context_is_disabled_by_default():
    backend = _load_rust_backend()
    assert backend is not None
    router = backend.PyPhotonicRouter(
        backend.GridSpec(20, 20, 1.0, 0.0, 0.0),
        backend.PrimitiveLibraryConfig(),
        backend.AStarConfig(),
    )

    cfg = router.crossing_config()
    assert cfg.enabled is False
    assert router.enforce_realized_crossing_validation() is True
    router.set_enforce_realized_crossing_validation(False)
    assert router.enforce_realized_crossing_validation() is False
    assert router.lidar_global_relaxed_repair_only() is False
    router.set_lidar_global_relaxed_repair_only(True)
    assert router.lidar_global_relaxed_repair_only() is True
    assert router.crossing_expected_count(1) == 0
    assert router.crossing_has_expected_pair(1, 2) is False
    assert router.crossing_allows_pair(1, 2) is False


def test_router_crossing_context_stores_expected_pairs():
    backend = _load_rust_backend()
    assert backend is not None
    router = backend.PyPhotonicRouter(
        backend.GridSpec(20, 20, 1.0, 0.0, 0.0),
        backend.PrimitiveLibraryConfig(),
        backend.AStarConfig(),
    )

    router.set_crossing_constraints(
        [
            backend.CrossingConstraint(1, 3, level=0, source_depth=1, target_depth=2),
            backend.CrossingConstraint(2, 3, level=1, source_depth=1, target_depth=2),
        ]
    )
    assert router.crossing_expected_count(3) == 2
    assert router.crossing_has_expected_pair(3, 1) is True
    assert router.crossing_allows_pair(3, 1) is False

    router.set_crossing_config(
        backend.CrossingConfig(
            enabled=True,
            crossing_loss=4.5,
            crossing_half_size_cells=3,
            min_straight_cells_per_crossing=6,
        )
    )
    assert router.crossing_allows_pair(3, 1) is True
    assert router.crossing_allows_pair(1, 2) is False
    assert len(router.crossing_constraints()) == 2

    router.clear_crossing_constraints()
    assert router.crossing_expected_count(3) == 0
    assert router.crossing_has_expected_pair(3, 1) is False


def test_crossing_enabled_route_uses_expected_partner_intersection():
    backend = _load_rust_backend()
    assert backend is not None

    primitive = backend.PrimitiveLibraryConfig()
    primitive.grid4_unit_grid = True
    router = backend.PyPhotonicRouter(
        backend.GridSpec(32, 32, 1.0, 0.0, 0.0),
        primitive,
        backend.AStarConfig(max_iterations=10_000),
    )

    router.route_single_net_and_commit(
        1,
        backend.State(10, 5, 2),
        backend.State(10, 24, 2),
        block_radius_cells=0,
    )
    router.set_crossing_constraints(
        [backend.CrossingConstraint(2, 1, level=0, source_depth=0, target_depth=1)]
    )
    router.set_crossing_config(
        backend.CrossingConfig(
            enabled=True,
            crossing_half_size_cells=0,
            min_straight_cells_per_crossing=2,
        )
    )

    route = router.route_single_net_and_commit(
        2,
        backend.State(3, 12, 0),
        backend.State(24, 12, 0),
        block_radius_cells=0,
    )

    net1_core = {tuple(cell) for cell in router.get_net_core_cells(1)}
    net2_core = {tuple(cell) for cell in router.get_net_core_cells(2)}
    assert net1_core & net2_core
    assert {int(cell[1]) for cell in route.cells} == {12}


def test_crossing_enabled_route_reserves_crossing_footprint():
    backend = _load_rust_backend()
    assert backend is not None

    primitive = backend.PrimitiveLibraryConfig()
    primitive.grid4_unit_grid = True
    router = backend.PyPhotonicRouter(
        backend.GridSpec(32, 32, 1.0, 0.0, 0.0),
        primitive,
        backend.AStarConfig(max_iterations=10_000),
    )

    router.route_single_net_and_commit(
        1,
        backend.State(10, 5, 2),
        backend.State(10, 24, 2),
        block_radius_cells=0,
    )
    router.set_crossing_constraints(
        [backend.CrossingConstraint(2, 1, level=0, source_depth=0, target_depth=1)]
    )
    router.set_crossing_config(
        backend.CrossingConfig(
            enabled=True,
            crossing_half_size_cells=2,
            min_straight_cells_per_crossing=2,
        )
    )

    route = router.route_single_net_and_commit(
        2,
        backend.State(3, 12, 0),
        backend.State(24, 12, 0),
        block_radius_cells=0,
    )

    static_cells = {tuple(cell) for cell in router.raw_static_obstacle_cells()}
    expected_reserved = {
        (x, y)
        for x in range(8, 13)
        for y in range(10, 15)
    }
    assert expected_reserved <= static_cells
    events = router.crossing_events()
    assert len(events) == 1
    assert events[0]["net_id"] == 2
    assert events[0]["partner_net_id"] == 1
    assert tuple(events[0]["point"]) == (10.0, 12.0)
    assert expected_reserved <= {tuple(cell) for cell in events[0]["reservation_cells"]}
    svg = router.export_debug_svg(route)
    assert 'id="crossing-events"' in svg
    assert 'class="crossing-event"' in svg
    assert 'data-net-id="2"' in svg
    assert 'data-partner-net-id="1"' in svg
    assert 'fill="#8a8a8a"' in svg
    assert 'stroke="#d81b60"' not in svg
    assert 'stroke="#00acc1"' not in svg

    assert router.ripup_route(2) is True
    assert router.crossing_events() == []
    static_cells_after_ripup = {tuple(cell) for cell in router.raw_static_obstacle_cells()}
    assert expected_reserved.isdisjoint(static_cells_after_ripup)


def test_crossing_enabled_route_fails_without_valid_crossing_geometry():
    backend = _load_rust_backend()
    assert backend is not None

    primitive = backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        straight_short_cells=1,
        straight_long_cells=4,
        bend_radius_cells=1,
        allow_45_degree_turns=True,
    )
    router = backend.PyPhotonicRouter(
        backend.GridSpec(32, 32, 1.0, 0.0, 0.0),
        primitive,
        backend.AStarConfig(max_iterations=10_000),
    )

    router.route_single_net_and_commit(
        1,
        backend.State(10, 12, 0),
        backend.State(12, 12, 0),
        block_radius_cells=0,
    )
    router.set_crossing_constraints(
        [backend.CrossingConstraint(2, 1, level=0, source_depth=0, target_depth=1)]
    )
    router.set_crossing_config(
        backend.CrossingConfig(
            enabled=True,
            crossing_half_size_cells=0,
                min_straight_cells_per_crossing=4,
        )
    )

    with pytest.raises(RuntimeError, match="No crossing-compliant route found"):
        router.route_single_net_and_commit(
            2,
            backend.State(11, 3, 2),
            backend.State(11, 24, 2),
            block_radius_cells=0,
        )

    assert router.get_net_core_cells(2) == []


def test_route_debug_svg_can_use_obstacle_snapshot():
    backend = _load_rust_backend()
    assert backend is not None

    primitive = backend.PrimitiveLibraryConfig()
    primitive.grid4_unit_grid = True
    router = backend.PyPhotonicRouter(
        backend.GridSpec(32, 32, 1.0, 0.0, 0.0),
        primitive,
        backend.AStarConfig(max_iterations=10_000),
    )

    first = router.route_single_net_and_commit(
        1,
        backend.State(3, 8, 0),
        backend.State(12, 8, 0),
        block_radius_cells=0,
    )
    router.route_single_net_and_commit(
        2,
        backend.State(3, 14, 0),
        backend.State(12, 14, 0),
        block_radius_cells=0,
    )
    router.add_port_open_cells([(4, 8)])

    final_svg = router.export_debug_svg(first)
    snapshot_svg = router.export_debug_svg_with_obstacle_cells(first, [])

    assert 'fill="#000000"' in final_svg
    assert 'fill="#000000"' not in snapshot_svg
    assert (
        '<rect class="port-access" x="4" y="23" width="1" height="1" '
        'fill="#d93025" opacity="0.38" />'
    ) in final_svg
    assert (
        '<rect class="port-access" x="4" y="23" width="1" height="1" '
        'fill="#d93025" opacity="0.38" />'
    ) in snapshot_svg
    assert '<rect x="3" y="23" width="1" height="1" fill="#1a73e8" />' in snapshot_svg
