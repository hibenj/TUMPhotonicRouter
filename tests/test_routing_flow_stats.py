from routing_flow import (
    RipupRerouteConfig,
    RoutingFlowStats,
    _format_debug_route_indices,
    _parse_debug_svg_selector,
    run_routing_flow,
)
import benchmark_metadata
from translation.route_rust_types import RouteAttemptRecord, RouteSearchSummary
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
    assert stats.route_attempts >= stats.net_count
    assert stats.expanded_states >= 0
    assert stats.generated_neighbors >= 0
    assert stats.heap_pushes >= 0
    assert stats.heap_pops >= 0

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
    assert stats_dict["route_attempts"] == stats.route_attempts
    assert stats_dict["expanded_states"] == stats.expanded_states


def test_debug_svg_selector_parses_boolean_and_all_modes():
    assert _parse_debug_svg_selector(False) == (False, None)
    assert _parse_debug_svg_selector(True) == (True, None)
    assert _parse_debug_svg_selector("all") == (True, None)


def test_debug_svg_selector_parses_1_based_route_indices():
    assert _parse_debug_svg_selector(5) == (True, {5})
    assert _parse_debug_svg_selector("5-10") == (True, {5, 6, 7, 8, 9, 10})
    assert _parse_debug_svg_selector("2,5-7,10") == (True, {2, 5, 6, 7, 10})
    assert _parse_debug_svg_selector(range(2, 5)) == (True, {2, 3, 4})


def test_format_debug_route_indices_collapses_ranges():
    assert _format_debug_route_indices({2, 5, 6, 7, 10}) == "2,5-7,10"


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
        ripup_reroute_config: object | None = None,
        **_kwargs: object,
    ):
        captured["obstacle_config"] = obstacle_config
        captured["ripup_reroute_config"] = ripup_reroute_config
        captured["collect_route_stats"] = _kwargs.get("collect_route_stats")
        routed_layout = SimpleNamespace(name="routed_layout_rust")
        routed_layout.write_gds = lambda *_args, **_kwargs: None
        return SimpleNamespace(
            routed_layout=routed_layout,
            debug_artifacts=SimpleNamespace(
                realization_grid_spec=(10, 10, 0.5, 0.0, 0.0),
                static_blocked_cells=set(),
                route_search_summary=RouteSearchSummary(
                    route_count=2,
                    route_attempts=3,
                    simple_route_count=1,
                    repair_count=1,
                    expanded_states=42,
                    generated_neighbors=99,
                    heap_pushes=50,
                    heap_pops=45,
                    skipped_duplicate_heap_entries=5,
                    stale_generation_heap_entries=3,
                    closed_heap_entries=2,
                    max_heap_size=12,
                    dense_search_states=1000,
                    dense_search_storage_bytes=18_125,
                    best_cost_updates=51,
                    parent_updates=50,
                    obstacle_clearance_checks=88,
                    footprint_checks=77,
                    footprint_rect_checks=66,
                    full_grid_fallbacks=1,
                    neighbor_generation_time_us=1000,
                    heap_operation_time_us=2000,
                    legality_check_time_us=3000,
                    reconstruction_time_us=4000,
                ),
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

    ripup_config = RipupRerouteConfig(enabled=True, max_rounds=2)
    run_routing_flow(
        "MMI8x4",
        debug_timing=False,
        show_klayout=False,
        ripup_reroute_config=ripup_config,
    )

    config = captured.get("obstacle_config")
    assert hasattr(config, "obstacle_mode")
    assert config.obstacle_mode == "bounding_boxes"
    assert config.clear_port_open_cells_from_static is False
    assert captured.get("ripup_reroute_config") is ripup_config
    assert captured.get("collect_route_stats") is False


def test_run_routing_flow_collects_route_summary_when_stats_requested(monkeypatch):
    captured = {}

    def fake_load_benchmark(_benchmark_name: str):
        return SimpleNamespace(
            netlist=SimpleNamespace(instances={"a": object()}, routes={"n0": object()}),
            placements={},
        )

    def fake_layout_from_schematic(_schematic: object):
        return SimpleNamespace(
            name="unrouted_layout",
            bbox=(0.0, 0.0, 10.0, 10.0),
        )

    def fake_load_metadata(_benchmark_name: str, schematic: object):  # noqa: ARG001
        return {"node_types": None, "internal_delays_um": None}

    def fake_route_match_and_realize(
        _layout: object,
        _schematic: object,
        **kwargs: object,
    ):
        captured["collect_route_stats"] = kwargs.get("collect_route_stats")
        routed_layout = SimpleNamespace(name="routed_layout_rust")
        routed_layout.write_gds = lambda *_args, **_kwargs: None
        return SimpleNamespace(
            routed_layout=routed_layout,
            debug_artifacts=SimpleNamespace(
                realization_grid_spec=(10, 20, 0.5, 0.0, 0.0),
                static_blocked_cells={(1, 1), (2, 2)},
                route_search_summary=RouteSearchSummary(
                    route_count=1,
                    route_attempts=2,
                    route_failures=1,
                    simple_route_count=0,
                    repair_count=1,
                    astar_elapsed_s=0.25,
                    expanded_states=123,
                    generated_neighbors=456,
                    heap_pushes=300,
                    heap_pops=250,
                    skipped_duplicate_heap_entries=7,
                    stale_generation_heap_entries=4,
                    closed_heap_entries=3,
                    max_heap_size=80,
                    dense_search_states=2000,
                    dense_search_storage_bytes=36_250,
                    best_cost_updates=301,
                    parent_updates=300,
                    obstacle_clearance_checks=99,
                    footprint_checks=88,
                    footprint_rect_checks=77,
                    full_grid_fallbacks=1,
                    neighbor_generation_time_us=11_000,
                    heap_operation_time_us=22_000,
                    legality_check_time_us=33_000,
                    reconstruction_time_us=44_000,
                ),
                route_attempt_records=[
                    RouteAttemptRecord(
                        attempt_index=1,
                        bucket_name="normal_route",
                        net_id=1,
                        route_index=1,
                        net_name="n0",
                        source="a,o1",
                        target="b,o1",
                        elapsed_s=0.25,
                        expanded_states=123,
                        generated_neighbors=456,
                        heap_pushes=300,
                        heap_pops=250,
                        skipped_duplicate_heap_entries=7,
                        stale_generation_heap_entries=4,
                        closed_heap_entries=3,
                        max_heap_size=80,
                        dense_search_states=2000,
                        dense_search_storage_bytes=36_250,
                        best_cost_updates=301,
                        parent_updates=300,
                        obstacle_clearance_checks=99,
                        footprint_rect_checks=77,
                    )
                ],
            ),
            path_length_analysis_info=None,
            meander_requirements_info=None,
            meander_insertion_report_info=None,
        )

    import routing_flow

    monkeypatch.setattr(routing_flow, "load_benchmark", fake_load_benchmark)
    monkeypatch.setattr(routing_flow, "layout_from_schematic", fake_layout_from_schematic)
    monkeypatch.setattr(routing_flow, "load_benchmark_metadata", fake_load_metadata)
    monkeypatch.setattr(routing_flow, "route_match_and_realize", fake_route_match_and_realize)

    stats = RoutingFlowStats()
    run_routing_flow(
        "FAKE",
        debug_timing=False,
        show_klayout=False,
        stats=stats,
    )

    assert captured["collect_route_stats"] is True
    assert stats.route_attempts == 2
    assert stats.route_failures == 1
    assert stats.repair_count == 1
    assert stats.expanded_states == 123
    assert stats.generated_neighbors == 456
    assert stats.heap_pushes == 300
    assert stats.heap_pops == 250
    assert stats.skipped_duplicate_heap_entries == 7
    assert stats.stale_generation_heap_entries == 4
    assert stats.closed_heap_entries == 3
    assert stats.max_heap_size == 80
    assert stats.dense_search_states == 2000
    assert stats.dense_search_storage_bytes == 36_250
    assert stats.best_cost_updates == 301
    assert stats.parent_updates == 300
    assert stats.obstacle_clearance_checks == 99
    assert stats.footprint_checks == 88
    assert stats.footprint_rect_checks == 77
    assert stats.full_grid_fallbacks == 1
    assert stats.neighbor_generation_time_s == 0.011
    assert stats.heap_operation_time_s == 0.022
    assert stats.legality_check_time_s == 0.033
    assert stats.reconstruction_time_s == 0.044
    zero_primitive_counters = {
        "straight_short": 0,
        "straight_long": 0,
        "bend45": 0,
        "bend90": 0,
    }
    assert stats.route_attempt_records == [
        {
            "attempt_index": 1,
            "bucket_name": "normal_route",
            "net_id": 1,
            "route_index": 1,
            "net_name": "n0",
            "source": "a,o1",
            "target": "b,o1",
            "elapsed_s": 0.25,
            "failed": False,
            "repair_round": None,
            "error": None,
            "total_length_um": None,
            "route_cells": 0,
            "used_simple_route": False,
            "expanded_states": 123,
            "generated_neighbors": 456,
            "heap_pushes": 300,
            "heap_pops": 250,
            "skipped_duplicate_heap_entries": 7,
            "stale_generation_heap_entries": 4,
            "closed_heap_entries": 3,
            "max_heap_size": 80,
            "dense_search_states": 2000,
            "dense_search_storage_bytes": 36_250,
            "best_cost_updates": 301,
            "parent_updates": 300,
            "obstacle_clearance_checks": 99,
            "window_attempts": 0,
            "last_window_min_x": 0,
            "last_window_max_x": 0,
            "last_window_min_y": 0,
            "last_window_max_y": 0,
            "last_window_area_cells": 0,
            "primitive_generated_by_class": zero_primitive_counters,
            "primitive_bounds_rejects_by_class": zero_primitive_counters,
            "primitive_closed_rejects_by_class": zero_primitive_counters,
            "primitive_cost_pruned_by_class": zero_primitive_counters,
            "primitive_footprint_checks_by_class": zero_primitive_counters,
            "primitive_footprint_rejects_by_class": zero_primitive_counters,
            "primitive_accepted_by_class": zero_primitive_counters,
            "footprint_checks": 0,
            "footprint_rect_checks": 77,
            "dense_grid_build_time_s": 0.0,
            "dense_grid_cells": 0,
            "neighbor_generation_time_s": 0.0,
            "heap_operation_time_s": 0.0,
            "legality_check_time_s": 0.0,
            "reconstruction_time_s": 0.0,
            "max_window_area_cells": 0,
            "used_full_grid_fallback": False,
            "diagnostics": {},
        }
    ]
