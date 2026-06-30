from routing_flow import (
    RipupRerouteConfig,
    RoutingFlowStats,
    SCRIPT_ELECTRICAL_BUS_WIDTH_UM,
    SCRIPT_ELECTRICAL_PAD_PITCH_UM,
    SCRIPT_ELECTRICAL_WIRE_WIDTH_UM,
    SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    _build_arg_parser,
    _format_debug_route_indices,
    load_benchmark,
    _parse_debug_svg_selector,
    run_routing_flow,
)
import inspect
import benchmark_metadata
import pytest
from pathlib import Path
from photonic_router.routing_layers import get_routing_obstacle_layers
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from translation.electrical import (
    DEFAULT_BONDPAD_WIDTH_UM,
    DEFAULT_BUS_WIDTH_UM,
    DEFAULT_COMMON_BUS_BONDPAD_LENGTH_UM,
    DEFAULT_COMMON_BUS_BONDPAD_WIDTH_UM,
    DEFAULT_PAD_PITCH_UM,
    DEFAULT_WIRE_WIDTH_UM,
    ElectricalRoutingConfig,
)
from translation.layout_from_schematic import layout_from_schematic
from translation.photonic_verification import verify_photonic_routing
from translation.route_rust import route_match_and_realize
from translation.route_rust_types import (
    DEFAULT_BEND_RADIUS_UM,
    DEFAULT_MEANDER_MAX_HEIGHT_UM,
    MeanderInsertionConfig,
    RouteAttemptRecord,
    RouteSearchSummary,
    bend_radius_cells_from_um,
)
from types import SimpleNamespace
from typing import cast


def test_default_bend_radius_is_ten_um_on_routing_grid():
    assert DEFAULT_BEND_RADIUS_UM == 10.0
    assert bend_radius_cells_from_um(None, grid_size_um=0.5) == 20
    assert bend_radius_cells_from_um(10.0, grid_size_um=0.5) == 20
    assert bend_radius_cells_from_um(3.0, grid_size_um=0.5) == 6


def test_path_length_meander_height_defaults_are_centralized():
    assert DEFAULT_MEANDER_MAX_HEIGHT_UM == 80.0
    assert (
        MeanderInsertionConfig().max_meander_height_um
        == DEFAULT_MEANDER_MAX_HEIGHT_UM
    )
    assert SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM == DEFAULT_MEANDER_MAX_HEIGHT_UM
    assert (
        inspect.signature(run_routing_flow)
        .parameters["path_length_meander_height_um"]
        .default
        == DEFAULT_MEANDER_MAX_HEIGHT_UM
    )
    assert (
        inspect.signature(route_match_and_realize)
        .parameters["path_length_meander_height_um"]
        .default
        == DEFAULT_MEANDER_MAX_HEIGHT_UM
    )


def test_electrical_width_defaults_are_centralized():
    config = ElectricalRoutingConfig()

    assert DEFAULT_WIRE_WIDTH_UM == 20.0
    assert DEFAULT_BUS_WIDTH_UM == 400.0
    assert DEFAULT_BONDPAD_WIDTH_UM == 80.0
    assert DEFAULT_COMMON_BUS_BONDPAD_WIDTH_UM == DEFAULT_BUS_WIDTH_UM
    assert DEFAULT_COMMON_BUS_BONDPAD_LENGTH_UM == DEFAULT_BUS_WIDTH_UM
    assert DEFAULT_PAD_PITCH_UM == (
        config.bondpad_width_um + config.bondpad_spacing_um
    )
    assert config.wire_width_um == DEFAULT_WIRE_WIDTH_UM
    assert config.bus_width_um == DEFAULT_BUS_WIDTH_UM
    assert config.bondpad_width_um == DEFAULT_BONDPAD_WIDTH_UM
    assert config.common_bus_bondpad_width_um == DEFAULT_COMMON_BUS_BONDPAD_WIDTH_UM
    assert config.common_bus_bondpad_length_um == DEFAULT_COMMON_BUS_BONDPAD_LENGTH_UM
    assert config.pad_pitch_um == DEFAULT_PAD_PITCH_UM
    assert SCRIPT_ELECTRICAL_WIRE_WIDTH_UM == DEFAULT_WIRE_WIDTH_UM
    assert SCRIPT_ELECTRICAL_BUS_WIDTH_UM == DEFAULT_BUS_WIDTH_UM
    assert SCRIPT_ELECTRICAL_PAD_PITCH_UM == DEFAULT_PAD_PITCH_UM


def test_routing_flow_populates_stats():
    stats = RoutingFlowStats()

    run_routing_flow(
        "TOY",
        show_unrouted=False,
        show_routed=False,
        show_static_obstacles_svg=False,
        enable_path_length_matching=False,
        path_length_match_outputs=False,
        allow_45_degree_turns=True,
        bend_radius_um=5.0,
        waveguide_clearance_um=0.0,
        stats=stats,
    )

    assert stats.benchmark_name == "TOY"
    assert stats.total_time_s > 0.0
    assert stats.instance_count == 5
    assert stats.net_count == 4
    assert stats.static_grid_width is not None
    assert stats.static_grid_height is not None
    assert stats.raw_blocked_cells is not None
    assert stats.blocked_cells is not None
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


def test_route_match_uses_rust_batch_path_when_repair_disabled():
    schematic = load_benchmark("TOY")
    unrouted_layout = layout_from_schematic(schematic)

    result = route_match_and_realize(
        unrouted_layout,
        schematic,
        enable_path_length_matching=False,
        allow_45_degree_turns=True,
        bend_radius_um=5.0,
        collect_route_stats=True,
        ripup_reroute_config=RipupRerouteConfig(enabled=False),
        obstacle_config=StaticObstacleMapConfig(clearance_um=0.0),
    )

    summary = result.debug_artifacts.route_search_summary
    assert summary.route_count == 4
    assert summary.route_attempts == 4
    assert summary.route_failures == 0
    assert summary.repair_count == 0
    assert len(result.debug_artifacts.route_attempt_records) == 4
    assert {
        record.bucket_name for record in result.debug_artifacts.route_attempt_records
    } == {"normal_route"}


@pytest.mark.parametrize(
    "benchmark_name",
    [
        "TOY",
        "clements_8x8",
        "heater_s",
        "heater_s_compact",
        "heater_s_mod",
        "mmi_heater",
        "mmi_heater_8x4",
        "mmi_heater_8x4_ripup_reroute",
    ],
)
def test_benchmarks_route_with_astar_only(benchmark_name):
    stats = RoutingFlowStats()

    run_routing_flow(
        benchmark_name,
        show_unrouted=False,
        show_routed=False,
        show_static_obstacles_svg=False,
        enable_path_length_matching=False,
        allow_45_degree_turns=True,
        enable_simple_routes=False,
        waveguide_clearance_um=0.0,
        collect_route_stats=True,
        stats=stats,
    )

    assert stats.net_count > 0
    assert stats.simple_route_count == 0
    assert stats.route_attempts >= stats.net_count


def _route_heater_s_mod_for_regression(waveguide_clearance_um: float):
    schematic = load_benchmark("heater_s_mod")
    unrouted_layout = layout_from_schematic(schematic)
    metadata = benchmark_metadata.load_benchmark_metadata(
        "heater_s_mod",
        schematic=schematic,
    )

    result = route_match_and_realize(
        unrouted_layout,
        schematic,
        enable_path_length_matching=True,
        path_length_match_outputs=True,
        node_types=metadata.get("node_types"),
        internal_delays_um=metadata.get("internal_delays_um"),
        debug_dir=None,
        debug_prefix="heater_s_mod",
        allow_45_degree_turns=False,
        bend_radius_um=10.0,
        max_iterations=5_000_000,
        routing_window_scale=0.05,
        collect_route_stats=True,
        include_heater_obstacles=True,
        obstacle_config=StaticObstacleMapConfig(
            grid_size_um=2.0,
            obstacle_mode="bounding_boxes",
            clearance_um=float(waveguide_clearance_um),
            heater_clearance_um=10.0,
            chip_add_x_um=0.0,
            chip_add_y_um=40.0,
            clear_port_open_cells_from_static=False,
        ),
    )
    return schematic, unrouted_layout, result


@pytest.mark.parametrize("waveguide_clearance_um", [3.0, 0.0])
def test_heater_s_mod_90_degree_plm_regression(waveguide_clearance_um):
    schematic, unrouted_layout, result = _route_heater_s_mod_for_regression(
        waveguide_clearance_um
    )
    route_summary = result.debug_artifacts.route_search_summary

    assert route_summary.route_count == 81
    assert route_summary.route_failures == 0
    assert route_summary.repair_count == 0
    assert route_summary.simple_route_count >= 60
    assert route_summary.full_grid_fallbacks == 0

    analysis = result.path_length_analysis_info
    assert analysis is not None
    acceptance = analysis["path_length_acceptance"]
    assert acceptance["passed"] is True
    assert acceptance["failed_group_count"] == 0
    assert acceptance["failed_edge_count"] == 0
    assert acceptance["max_physical_residual_um"] == pytest.approx(0.0, abs=1e-9)

    meander_report = result.meander_insertion_report_info
    assert meander_report is not None
    assert meander_report["unmatched_length_um"] == pytest.approx(0.0, abs=1e-9)

    assert result.debug_artifacts.realization_grid_spec is not None
    verification = verify_photonic_routing(
        result.routed_layout,
        schematic,
        routed_net_records=result.debug_artifacts.routed_net_records,
        unrouted_layout=unrouted_layout,
        obstacle_layers=get_routing_obstacle_layers(include_heaters=True),
        realization_grid_spec=result.debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=result.debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=result.debug_artifacts.realization_bend_radius_cells,
    )
    assert verification.success, verification.as_dict()


def test_routing_flow_routes_single_heater_electrical_metal_end_to_end():
    routed = run_routing_flow(
        "mmi_heater",
        show_unrouted=False,
        show_routed=False,
        show_static_obstacles_svg=False,
        include_heater_obstacles=True,
        enable_electrical_routing=True,
    )

    assert "electrical_routing" in routed.info
    summary = routed.info["electrical_routing"]
    assert summary["terminal_group_count"] == 1
    assert summary["common_bus_success"] is True
    assert summary["pad_assignment_count"] >= 2
    assert summary["detailed_route_count"] >= 1
    assert summary["failed_detailed_route_count"] == 0
    assert summary["verification_success"] is True
    assert summary["verification_error_count"] == 0
    assert summary["verification_warning_count"] == 0
    assert summary["verification_issue_counts"] == {}
    assert summary["debug_artifact_count"] == 0
    assert summary["config"]["pad_side"] == "top"
    assert summary["config"]["bus_side"] == "bottom"
    assert summary["config"]["wire_width_um"] == 20.0
    assert summary["config"]["bus_width_um"] == DEFAULT_BUS_WIDTH_UM
    assert summary["config"]["terminal_contact_width_um"] == 10.0
    assert summary["config"]["metal_layer"] == (125, 0)
    assert summary["config"]["pad_marker_layer"] == (150, 0)
    assert routed.get_polygons(by="tuple").get((125, 0), [])
    assert routed.get_polygons(by="tuple").get((150, 0), [])


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


def test_electrical_cli_flags_parse_into_namespace():
    args = _build_arg_parser().parse_args(
        [
            "mmi_heater",
            "--electrical-routing",
            "true",
            "--electrical-pad-side",
            "bottom",
            "--electrical-grid-pitch-um",
            "20",
            "--electrical-obstacle-clearance-um",
            "5",
            "--electrical-wire-width-um",
            "18",
            "--electrical-bus-width-um",
            "24",
            "--electrical-terminal-contact-width-um",
            "32",
            "--electrical-pad-pitch-um",
            "160",
            "--proactive-congestion-weight",
            "3.5",
            "--proactive-congestion-radius-cells",
            "4",
            "--verbose-routes",
        ]
    )

    assert args.benchmark == "mmi_heater"
    assert args.electrical_routing is True
    assert args.electrical_pad_side == "bottom"
    assert args.electrical_grid_pitch_um == 20.0
    assert args.electrical_obstacle_clearance_um == 5.0
    assert args.electrical_wire_width_um == 18.0
    assert args.electrical_bus_width_um == 24.0
    assert args.electrical_terminal_contact_width_um == 32.0
    assert args.electrical_pad_pitch_um == 160.0
    assert args.proactive_congestion_weight == 3.5
    assert args.proactive_congestion_radius_cells == 4
    assert args.verbose_routes is True


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
        captured["bend_radius_um"] = _kwargs.get("bend_radius_um")
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
        bend_radius_um=3.0,
        ripup_reroute_config=ripup_config,
    )

    config = captured.get("obstacle_config")
    assert config is not None
    assert hasattr(config, "obstacle_mode")
    assert config.obstacle_mode == "bounding_boxes"
    assert config.clear_port_open_cells_from_static is False
    assert captured.get("ripup_reroute_config") is ripup_config
    assert captured.get("collect_route_stats") is False
    assert captured.get("bend_radius_um") == 3.0


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
                    crossing_candidate_checks=12,
                    crossing_accepted=3,
                    crossing_reject_not_perpendicular=4,
                    crossing_reject_margin=5,
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
                        crossing_candidate_checks=12,
                        crossing_accepted=3,
                        crossing_reject_not_perpendicular=4,
                        crossing_reject_margin=5,
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
    assert stats.crossing_candidate_checks == 12
    assert stats.crossing_accepted == 3
    assert stats.crossing_reject_not_perpendicular == 4
    assert stats.crossing_reject_margin == 5
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
            "crossing_candidate_checks": 12,
            "crossing_accepted": 3,
            "crossing_reject_non_straight": 0,
            "crossing_reject_not_perpendicular": 4,
            "crossing_reject_margin": 5,
            "crossing_reject_wrong_order": 0,
            "crossing_reject_unexpected_owner": 0,
            "crossing_reject_unmatched_owner": 0,
            "crossing_reject_unmatched_centerline": 0,
            "crossing_reject_unmatched_footprint": 0,
            "crossing_reject_pending_straight": 0,
            "route_search_total_time_s": 0.0,
            "dense_grid_build_time_s": 0.0,
            "dense_grid_cells": 0,
            "search_loop_time_s": 0.0,
            "obstacle_map_prepare_time_s": 0.0,
            "simple_route_time_s": 0.0,
            "commit_prepare_time_s": 0.0,
            "commit_time_s": 0.0,
            "neighbor_generation_time_s": 0.0,
            "heap_operation_time_s": 0.0,
            "legality_check_time_s": 0.0,
            "reconstruction_time_s": 0.0,
            "max_window_area_cells": 0,
            "used_full_grid_fallback": False,
            "diagnostics": {},
        }
    ]


def test_run_routing_flow_can_append_electrical_routing(monkeypatch):
    captured = {}

    schematic = SimpleNamespace(
        netlist=SimpleNamespace(
            instances={"heater_0": object()},
            routes={"n0": object()},
        ),
        placements={},
    )
    optical_layout = SimpleNamespace(
        name="unrouted_layout",
        bbox=(0.0, 0.0, 10.0, 10.0),
    )
    optical_routed_layout = SimpleNamespace(
        name="optical_routed",
        info={"optical_marker": "kept"},
    )
    electrical_layout = SimpleNamespace(
        name="electrical_routed",
        info={},
    )
    electrical_layout.write_gds = lambda path: captured.setdefault("gds_path", path)

    def fake_load_benchmark(_benchmark_name: str):
        return schematic

    def fake_layout_from_schematic(_schematic: object):
        return optical_layout

    def fake_load_metadata(_benchmark_name: str, schematic: object):  # noqa: ARG001
        return {"node_types": None, "internal_delays_um": None}

    def fake_route_match_and_realize(
        _layout: object,
        _schematic: object,
        **_kwargs: object,
    ):
        return SimpleNamespace(
            routed_layout=optical_routed_layout,
            debug_artifacts=SimpleNamespace(
                obstacle_svg=None,
                route_svgs=[],
                realization_grid_spec=(10, 10, 0.5, 0.0, 0.0),
                static_blocked_cells=set(),
                route_attempt_records=[],
                route_search_summary=RouteSearchSummary(route_count=1),
            ),
            path_length_analysis_info=None,
            meander_requirements_info=None,
            meander_insertion_report_info=None,
            pipeline_timings_s={},
        )

    electrical_config = SimpleNamespace(name="electrical_config")

    def fake_route_electrical_heaters(
        component: object,
        schematic_arg: object,
        config: object,
        *,
        debug_dir: object,
        debug_prefix: str,
    ):
        captured["electrical_component_input"] = component
        captured["electrical_schematic"] = schematic_arg
        captured["electrical_config"] = config
        captured["debug_dir"] = debug_dir
        captured["debug_prefix"] = debug_prefix
        return SimpleNamespace(
            terminal_groups=(object(), object()),
            common_bus=SimpleNamespace(success=True, failed_heaters=()),
            pad_plan=SimpleNamespace(assignments=(object(), object(), object())),
            common_bus_escape=SimpleNamespace(success=True),
            detailed_bundle_routes=SimpleNamespace(
                routes=(object(), object()),
                failed_routes=(),
            ),
            routed_component=electrical_layout,
            debug_artifacts={
                "common_bus_svg": "build/electrical/fake_common_bus.svg",
                "metal_snapshot_svg": "build/electrical/fake_metal_snapshot.svg",
            },
        )

    import routing_flow

    monkeypatch.setattr(routing_flow, "load_benchmark", fake_load_benchmark)
    monkeypatch.setattr(
        routing_flow,
        "layout_from_schematic",
        fake_layout_from_schematic,
    )
    monkeypatch.setattr(routing_flow, "load_benchmark_metadata", fake_load_metadata)
    monkeypatch.setattr(
        routing_flow,
        "route_match_and_realize",
        fake_route_match_and_realize,
    )
    monkeypatch.setattr(
        routing_flow,
        "route_electrical_heaters",
        fake_route_electrical_heaters,
    )

    stats = RoutingFlowStats()
    result = run_routing_flow(
        "FAKE",
        debug_timing=False,
        debug_svgs=True,
        show_klayout=False,
        enable_electrical_routing=True,
        electrical_config=cast(ElectricalRoutingConfig, cast(object, electrical_config)),
        stats=stats,
    )

    assert result is electrical_layout
    assert captured["electrical_component_input"] is optical_routed_layout
    assert captured["electrical_schematic"] is schematic
    assert captured["electrical_config"] is electrical_config
    assert captured["debug_dir"] == Path("build")
    assert captured["debug_prefix"] == "fake"
    assert captured["gds_path"] == "build/routed_FAKE.gds"
    assert electrical_layout.info["optical_marker"] == "kept"
    summary = electrical_layout.info["electrical_routing"]
    assert summary["terminal_group_count"] == 2
    assert summary["common_bus_success"] is True
    assert summary["pad_assignment_count"] == 3
    assert summary["detailed_route_count"] == 2
    assert summary["failed_detailed_route_count"] == 0
    assert summary["verification_issue_counts"] == {}
    assert summary["debug_artifact_count"] == 2
    assert summary["debug_artifacts"] == {
        "common_bus_svg": "build/electrical/fake_common_bus.svg",
        "metal_snapshot_svg": "build/electrical/fake_metal_snapshot.svg",
    }
    assert stats.electrical_terminal_groups == 2
    assert stats.electrical_pad_assignments == 3
    assert stats.electrical_detailed_routes == 2
    assert stats.electrical_failed_detailed_routes == 0
    assert stats.step_times_s["electrical_routing"] >= 0.0
    stats_dict = stats.as_dict()
    assert stats_dict["electrical_terminal_groups"] == 2
    assert stats_dict["electrical_pad_assignments"] == 3
    assert stats_dict["electrical_detailed_routes"] == 2
    assert stats_dict["electrical_failed_detailed_routes"] == 0
