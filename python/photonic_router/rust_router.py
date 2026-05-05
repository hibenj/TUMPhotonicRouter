"""Helper wrapper for Rust photonic router bindings."""

from photonic_router_rust import (
    AStarConfig,
    GridSpec,
    PrimitiveLibraryConfig,
    PyPhotonicRouter,
    State,
)


def demo_route():
    grid = GridSpec(80, 60, 0.5, 0.0, 0.0)
    primitive = PrimitiveLibraryConfig()
    astar = AStarConfig(max_iterations=200000, bend_weight=1.0, target_tolerance_cells=0)
    router = PyPhotonicRouter(grid, primitive, astar)
    blocked = [(20, y) for y in range(10, 50)]
    router.set_static_cells(blocked)
    source = State(5, 20, 0)
    target = State(70, 20, 0)
    route = router.route_single_net(source, target, opened_cells=[(20, 20)])
    return route, router.export_debug_svg(route)
