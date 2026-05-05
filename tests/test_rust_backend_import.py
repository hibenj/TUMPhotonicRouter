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

