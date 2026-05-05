from routing_flow import RoutingFlowStats, run_routing_flow


def test_routing_flow_populates_stats():
    stats = RoutingFlowStats()

    run_routing_flow(
        "TOY",
        show_unrouted=False,
        show_routed=False,
        show_static_obstacles_svg=False,
        stats=stats,
    )

    assert stats.benchmark_name == "TOY"
    assert stats.total_time_s > 0.0
    assert stats.instance_count == 5
    assert stats.net_count == 4
    assert stats.static_grid_width is not None
    assert stats.static_grid_height is not None
    assert stats.raw_blocked_cells > 0
    assert stats.blocked_cells > 0
    assert stats.port_open_cells > 0

    for step_name in [
        "load_benchmark",
        "layout_from_schematic",
        "build_static_obstacle_map",
        "baseline_gdsfactory_routing",
    ]:
        assert step_name in stats.step_times_s
        assert stats.step_times_s[step_name] >= 0.0

    stats_dict = stats.as_dict()
    assert stats_dict["benchmark_name"] == "TOY"
    assert stats_dict["total_time_s"] == stats.total_time_s
