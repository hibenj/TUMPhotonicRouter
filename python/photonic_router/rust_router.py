"""Helper wrapper for Rust photonic router bindings."""

from .static_obstacle_builder import _load_rust_backend


def demo_route():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError("Rust backend is not available")

    grid = rust_backend.GridSpec(80, 60, 0.5, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig()
    astar = rust_backend.AStarConfig(
        max_iterations=200000, bend_weight=1.0, target_tolerance_cells=0
    )
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    blocked = [(20, y) for y in range(10, 50)]
    router.set_static_cells(blocked)
    source = rust_backend.State(5, 20, 0)
    target = rust_backend.State(70, 20, 0)
    route = router.route_single_net(source, target, opened_cells=[(20, 20)])
    return route, router.export_debug_svg(route)
