from routing_flow import RoutingFlowStats, run_routing_flow
import benchmark_metadata
from types import SimpleNamespace


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


def test_resolve_auto_internal_delay_markers_per_instance(monkeypatch):
    class _Instance:
        def __init__(self, component_name: str):
            self.component = component_name

    class _Netlist:
        instances = {
            "heater_0": _Instance("heater_comp"),
            "mmi_0": _Instance("mmi_comp"),
        }

    class _Schematic:
        netlist = _Netlist()
        placements = {}

    monkeypatch.setattr(
        benchmark_metadata,
        "component_internal_delay_um",
        lambda component_name: 88.0 if component_name == "heater_comp" else None,
    )

    delays = benchmark_metadata.resolve_internal_delays_for_instances(
        _Schematic(),
        {"heater_0": "auto", "mmi_0": 0.0},
    )

    assert delays["heater_0"] == 88.0
    assert delays["mmi_0"] == 0.0


def test_component_internal_delay_reads_kfactory_info_attributes():
    assert benchmark_metadata.component_internal_delay_um("straight_heater_metal") == 320.0


def test_run_routing_flow_uses_strict_default_obstacle_config(monkeypatch):
    captured = {}

    def fake_load_benchmark(_benchmark_name: str):
        return SimpleNamespace(
            netlist=SimpleNamespace(instances={}, routes={}),
            placements={},
        )

    def fake_layout_from_schematic(_schematic: object):
        return SimpleNamespace(
            name="unrouted_layout",
            bbox=(0.0, 0.0, 10.0, 10.0),
        )

    def fake_load_metadata(_benchmark_name: str, schematic: object):  # noqa: ARG001
        return {
            "node_types": None,
            "internal_delays_um": None,
        }

    def fake_route_match_and_realize(
        _layout: object,
        _schematic: object,
        *,
        obstacle_config: object | None = None,
        **_kwargs: object,
    ):
        captured["obstacle_config"] = obstacle_config
        routed_layout = SimpleNamespace(name="routed_layout_rust")
        routed_layout.write_gds = lambda *_args, **_kwargs: None
        return SimpleNamespace(
            routed_layout=routed_layout,
            debug_artifacts=SimpleNamespace(
                realization_grid_spec=(10, 10, 0.5, 0.0, 0.0),
                static_blocked_cells=set(),
            ),
            path_length_analysis_info=None,
            meander_requirements_info=None,
            meander_insertion_report_info=None,
        )

    import routing_flow

    monkeypatch.setattr(routing_flow, "load_benchmark", fake_load_benchmark)
    monkeypatch.setattr(routing_flow, "layout_from_schematic", fake_layout_from_schematic)
    monkeypatch.setattr(
        routing_flow, "load_benchmark_metadata", fake_load_metadata
    )
    monkeypatch.setattr(
        routing_flow, "route_match_and_realize", fake_route_match_and_realize
    )

    run_routing_flow("MMI8x4", debug_timing=False, show_klayout=False)

    config = captured.get("obstacle_config")
    assert hasattr(config, "obstacle_mode")
    assert config.obstacle_mode == "bounding_boxes"
    assert config.clear_port_open_cells_from_static is False
