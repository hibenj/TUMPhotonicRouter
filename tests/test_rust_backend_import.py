import sys
from pathlib import Path

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


def test_crossing_enabled_route_uses_expected_partner_anchor():
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
