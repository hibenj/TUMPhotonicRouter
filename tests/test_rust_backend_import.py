import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from photonic_router.static_obstacle_builder import _load_rust_backend


def test_rust_backend_exposes_router_class():
    backend = _load_rust_backend()
    assert backend is not None
    assert hasattr(backend, "PyPhotonicRouter")
    assert hasattr(backend, "GridSpec")
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
