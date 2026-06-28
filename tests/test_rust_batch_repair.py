import pytest

from photonic_router.static_obstacle_builder import _load_rust_backend


def _build_lane_repair_router():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for native batch repair regression.")

    width = 50
    height = 30
    grid = rust_backend.GridSpec(width, height, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        straight_short_cells=1,
        straight_long_cells=4,
        bend_radius_cells=1,
        allow_45_degree_turns=False,
    )
    primitive.grid4_unit_grid = True
    astar = rust_backend.AStarConfig(
        max_iterations=300_000,
        use_routing_window=False,
        require_target_angle=True,
        collect_detailed_timing=True,
    )
    astar.enable_simple_routes = False
    astar.heuristic_mode = "distance"

    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)

    open_cells = set()
    for x in range(2, 48):
        open_cells.add((x, 10))
        open_cells.add((x, 14))
    for y in range(10, 15):
        open_cells.add((2, y))
        open_cells.add((47, y))
    static_cells = [
        (x, y)
        for x in range(width)
        for y in range(height)
        if (x, y) not in open_cells
    ]
    router.set_static_cells(static_cells)
    return rust_backend, router


def test_rust_batch_repair_rips_and_reroutes_dynamic_blocker():
    rust_backend, router = _build_lane_repair_router()
    state = rust_backend.State
    jobs = [
        (1, state(2, 10, 0), state(47, 10, 0), [], []),
        (2, state(5, 10, 0), state(44, 10, 0), [], []),
    ]

    result = router.route_many_with_repair_and_commit(
        jobs,
        0,
        0,
        0,
        4,
        4,
        10.0,
        5,
    )

    assert result["status"] == "routed"
    assert result["failed_net_id"] is None
    assert result["error"] is None
    assert result["repair_count"] == 1

    attempts = result["attempts"]
    assert [attempt["bucket_name"] for attempt in attempts] == [
        "normal_route",
        "normal_route",
        "probe_route",
        "repair_failed_net",
        "reroute_victims",
    ]
    assert attempts[1]["net_id"] == 2
    assert attempts[1]["failed"] is True
    assert attempts[2]["net_id"] == 2
    assert attempts[2]["failed"] is False
    assert attempts[3]["candidate_blockers"] == [1]
    assert attempts[3]["ripup_ids"] == [1]
    assert attempts[4]["net_id"] == 1
    assert attempts[4]["failed"] is False

    routes_by_net = {entry["net_id"]: entry["route"] for entry in result["routes"]}
    assert set(routes_by_net) == {1, 2}
    assert any(y == 14 for _x, y in routes_by_net[1].cells)
    assert all(y == 10 for _x, y in routes_by_net[2].cells)

    assert router.get_net_cells(1)
    assert router.get_net_cells(2)
