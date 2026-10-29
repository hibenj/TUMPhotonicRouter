from __future__ import annotations

from dataclasses import replace
from typing import Any, Protocol, cast

import pytest
from gdsfactory.component import Component
from photonic_router import flow as routing_flow
from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
    MissingLengthRequirement,
    PortRef,
    RoutedEdgeKey,
    build_graph_from_schematic,
)
from photonic_router.static_obstacle_builder import _load_rust_backend

import routing_flow_optical
import translation.route_rust as route_rust
from tests.fixtures.path_length import (
    FakePlanner,
    fake_backend,
    meander_report,
    planned_meander,
    planned_result,
    real_route_obj,
    search_config,
)
from tests.fixtures.synthetic_layouts import (
    PATH_LENGTH_INTERNAL_DELAYS_UM,
    PATH_LENGTH_NODE_TYPES,
    path_length_schematic,
)
from translation.path_length_diagnostics import (
    analysis_to_info_dict,
    format_path_length_acceptance_failure,
    matching_group_diagnostics_to_info,
    output_matching_diagnostics_to_info,
    path_length_acceptance_summary,
)
from translation.path_length_requirements import (
    build_requirement_delay_candidates,
    compute_group_lifted_requirements,
    compute_output_matching_requirements,
    merge_missing_length_requirements,
    minimum_four_bend_extra_length_um,
)
from translation.route_rust import (
    MeanderInsertionConfig,
    RoutedNetRecord,
    RouteRustPipelineResult,
    RustRouteDebugArtifacts,
    analyze_meander_insertion_for_requirements,
    analyze_path_length_matching,
)
from translation.route_rust_meanders import (
    _axis_aligned_centerline_run_lengths_um,
    _build_planner_context,
    _CandidateWorkItem,
    _ContextProfile,
    _meander_search_config,
    _MeanderPlannerContext,
    _normalize_minimum_insertable_request,
)
from translation.route_rust_types import MeanderEdgeReport, MeanderInsertionReport
from translation.routing import api as routing_api


class _SchematicLike(Protocol):
    netlist: Any
    placements: dict[str, object]


def test_planner_context_uses_static_handle_when_registration_fast_path_is_unavailable():
    class _RouteObj:
        cells = [(4, 4)]

    class _StaticHandle:
        def cells(self) -> list[tuple[int, int]]:
            return [(1, 2), (3, 4)]

    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=_RouteObj(),
        total_length_um=10.0,
    )

    context = _build_planner_context(
        rust_backend=fake_backend(),
        routed_net_records=[record],
        realization_grid_spec=(16, 16, 1.0, 0.0, 0.0),
        allow_45_degree_turns=False,
        bend_radius_cells=4,
        static_blocked_cells=None,
        static_blocked_cell_handle=_StaticHandle(),
        debug_meanders=True,
    )

    assert context.base_static_cells == {(1, 2), (3, 4)}
    assert set(cast(Any, context.planner).router.static_cells) == {(1, 2), (3, 4)}
    assert context.profile is not None
    assert context.profile.setup_profile["base_static_from_handle"] == 1.0
    assert context.profile.setup_profile["base_static_cell_count"] == 2.0


def test_planner_context_avoids_python_route_cell_bookkeeping_without_registration():
    class _RouteObj:
        cells = [(4, 4)]

    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=_RouteObj(),
        total_length_um=10.0,
    )

    context = _build_planner_context(
        rust_backend=fake_backend(),
        routed_net_records=[record],
        realization_grid_spec=(16, 16, 1.0, 0.0, 0.0),
        allow_45_degree_turns=False,
        bend_radius_cells=4,
        static_blocked_cells=[(3, 3)],
        route_clearance_radius_cells=1,
        debug_meanders=True,
    )

    assert set(cast(Any, context.planner).router.static_cells) == {(3, 3)}
    assert context.route_occupancy_radius_cells == 1
    assert context.profile is not None
    assert context.profile.setup_profile["route_occupancy_radius_cells"] == 1.0
    assert context.profile.setup_profile["meander_box_clearance_radius_cells"] == 0.0
    assert context.profile.setup_profile["registered_route_cell_acceleration_enabled"] == 0.0
    assert not hasattr(context, "route_cells_by_edge")
    assert not hasattr(context, "base_open_cells_for_edge")


def test_planner_context_builds_no_profile_without_debug_meanders():
    class _RouteObj:
        cells = [(4, 4)]

    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )
    context = _build_planner_context(
        rust_backend=fake_backend(),
        routed_net_records=[
            RoutedNetRecord(
                net_name=edge.net_name,
                source=edge.source,
                target=edge.target,
                route_obj=_RouteObj(),
                total_length_um=10.0,
            )
        ],
        realization_grid_spec=(16, 16, 1.0, 0.0, 0.0),
        allow_45_degree_turns=False,
        bend_radius_cells=4,
        static_blocked_cells=[(3, 3)],
    )

    assert context.profile is None


def test_axis_aligned_centerline_run_lengths_are_sorted_descending():
    centerline = (
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 5.0),
        (50.0, 5.0),
        (55.0, 10.0),
        (55.0, 25.0),
    )

    assert _axis_aligned_centerline_run_lengths_um(centerline) == [
        30.0,
        20.0,
        15.0,
        5.0,
    ]


def test_requirement_request_carries_the_split_fallback_to_rust():
    """The selection pass sends one vector per requirement: the candidates with
    their per-edge totals and the requiring edge as the split fallback; Rust
    answers which branch planned (here the three-chunk split)."""
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )

    def _plan_requests(
        _candidate_geometry_indices: list[list[int]],
        _candidate_requested_extra_lengths_um: list[list[float]],
        **_kwargs: object,
    ) -> dict[str, object]:
        plans = [{"inserted_extra_length_um": 40.0, "endpoint_inset_um": 0.0}] * 3
        return {
            "status": "planned",
            "planning_mode": "rust_registered_split_route_runs",
            "selected_candidate_index": 2,
            "split_part_count": 3,
            "plans": plans,
            "candidate_results": [
                {"candidate_index": 0, "status": "no_candidate", "reason": "x", "plans": []},
                {"candidate_index": 1, "status": "no_candidate", "reason": "y", "plans": []},
                {"candidate_index": 2, "status": "planned", "reason": "", "plans": plans},
            ],
        }

    fake_planner = FakePlanner(None, plan_requests=_plan_requests)
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=object(),
        total_length_um=90.0,
    )
    context = _MeanderPlannerContext(
        planner=cast(Any, fake_planner),
        by_edge={edge: record},
        updated={},
        registered_geometry_index_by_edge={edge: 42},
        max_bumps_by_edge={edge: 5},
        base_static_cells=set(),
        grid_size_um=1.0,
        bend_radius_um=10.0,
    )
    direct = DelayInsertionCandidate(
        requirement_edge_key=edge,
        edge_keys=(edge,),
        extra_length_um=120.0,
        reason="direct_edge",
        affected_requirement_edge_keys=(edge,),
    )

    outcome = context.plan_requirement_requests_registered(
        work_items=[
            _CandidateWorkItem(
                candidate=direct,
                affected_edges=(edge,),
                requests_by_edge={edge: 120.0},
                credit_extra_length=120.0,
                existing_physical_extra_length=0.0,
            )
        ],
        split_edge_key=edge,
        split_requested=120.0,
        min_insertable_extra_um=30.0,
        search_config=search_config(),
        auto_endpoint_inset_um=None,
    )

    assert fake_planner.calls == [
        {
            "candidate_geometry_indices": [[42]],
            "candidate_requested_extra_lengths_um": [[120.0]],
            "split_geometry_index": 42,
            "split_requested_extra_length_um": 120.0,
            "min_insertable_extra_length_um": 30.0,
            "max_split_parts": 8,
            "min_bend_radius_um": None,
            "min_straight_um": 1.0,
            "max_meander_height_um": 80.0,
            "min_segment_length_um": 1.0,
            "auto_endpoint_inset_um": None,
            "clearance_radius_cells": 0,
            "side_policy": "both",
            "planning_mode": "fill_box_multi_bump",
        }
    ]
    assert outcome.selected_work_index is None
    assert outcome.split_part_count == 3
    assert [planned_edge for planned_edge, *_ in outcome.plans] == [edge, edge, edge]
    assert outcome.edge_calls == 5  # two failed candidates, three planned chunks


def test_build_graph_from_schematic_tracks_port_directions_and_fanout_shape():
    schematic = path_length_schematic()
    graph = build_graph_from_schematic(schematic, node_types=PATH_LENGTH_NODE_TYPES)

    assert graph.nodes["gc_0"].node_type.value == "input"
    assert graph.nodes["mmi_0"].node_type.value == "gate"
    assert graph.nodes["gc_2"].node_type.value == "output"

    key = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    assert key in graph.edges
    assert graph.nodes["gc_0"].ports["o1"].direction.value == "output"
    assert graph.nodes["mmi_0"].ports["o2"].direction.value == "input"


def test_missing_length_analysis_balances_multi_input_node():
    schematic = path_length_schematic()
    records = [
        RoutedNetRecord(
            net_name="gc0_to_mmi_in1",
            source=PortRef(instance="gc_0", port="o1"),
            target=PortRef(instance="mmi_0", port="o2"),
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    edge_long = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )

    assert result.edge_missing_lengths_um[edge_short] == 20.0
    assert result.edge_missing_lengths_um[edge_long] == 0.0
    assert any(req.edge_key == edge_short for req in result.requirements)


def test_path_length_analysis_reports_matching_groups():
    schematic = path_length_schematic()
    records = [
        RoutedNetRecord(
            net_name="gc0_to_mmi_in1",
            source=PortRef(instance="gc_0", port="o1"),
            target=PortRef(instance="mmi_0", port="o2"),
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    info = analysis_to_info_dict(result)
    groups = cast(list[dict[str, object]], info["matching_groups"])

    assert len(groups) == 1
    assert groups[0]["node_name"] == "mmi_0"
    assert groups[0]["incoming_count"] == 2
    assert groups[0]["edges_requiring_meander"] == 1
    assert groups[0]["max_missing_length_um"] == pytest.approx(20.0)


def test_group_lifted_requirements_raise_sub_bump_deficit_to_reachable_target():
    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=None,
            total_length_um=99.5,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    requirements, groups = compute_group_lifted_requirements(
        result,
        minimum_insertable_extra_um=25.0,
    )
    required_by_edge = {req.edge_key: req.missing_length_um for req in requirements}

    assert required_by_edge[short_edge] == pytest.approx(25.5)
    assert required_by_edge[long_edge] == pytest.approx(25.0)
    assert groups[0]["target_lift_um"] == pytest.approx(25.0)
    assert groups[0]["edges_requiring_meander"] == 2


def test_group_lifted_requirements_do_not_raise_already_reachable_deficits():
    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=None,
            total_length_um=70.0,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    requirements, groups = compute_group_lifted_requirements(
        result,
        minimum_insertable_extra_um=25.0,
    )

    assert requirements == [MissingLengthRequirement(edge_key=short_edge, missing_length_um=30.0)]
    assert groups[0]["target_lift_um"] == pytest.approx(0.0)
    assert groups[0]["edges_requiring_meander"] == 1


def test_route_match_and_realize_plans_lifted_sub_bump_group(monkeypatch):
    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=object(),
            total_length_um=99.5,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=object(),
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=object(),
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=object(),
            total_length_um=60.0,
        ),
    ]
    routed_layout = Component(name="lifted_pipeline")
    captured: dict[str, object] = {}

    monkeypatch.setattr(
        routing_api,
        "route_nets_rust",
        lambda *args, **kwargs: (
            routed_layout,
            RustRouteDebugArtifacts(
                obstacle_svg=None,
                route_svgs=[],
                routed_edge_lengths_um={},
                routed_net_records=records,
                realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
                realization_bend_radius_cells=4,
            ),
        ),
    )

    class _ObstacleMap:
        blocked_cells: tuple[tuple[int, int], ...] = ()

    monkeypatch.setattr(
        routing_api,
        "build_static_obstacle_map",
        lambda *args, **kwargs: _ObstacleMap(),
    )

    def _fake_meander_planner(
        routed_net_records: list[RoutedNetRecord],
        requirements: list[MissingLengthRequirement],
        **_kwargs: object,
    ) -> tuple[list[RoutedNetRecord], MeanderInsertionReport]:
        required_by_edge = {req.edge_key: req.missing_length_um for req in requirements}
        assert required_by_edge[short_edge] == pytest.approx(25.5)
        assert required_by_edge[long_edge] == pytest.approx(25.0)
        captured["planner_requirements"] = requirements
        updated = [
            replace(
                record,
                meander_auto_plan={
                    "requested_extra_length_um": required_by_edge[
                        RoutedEdgeKey(
                            net_name=record.net_name,
                            source=record.source,
                            target=record.target,
                        )
                    ],
                    "selected_meander_centerline": [(0.0, 0.0), (1.0, 0.0)],
                    "selected_run_start_index": 0,
                    "selected_run_end_index": 1,
                },
            )
            if RoutedEdgeKey(
                net_name=record.net_name,
                source=record.source,
                target=record.target,
            )
            in required_by_edge
            else record
            for record in routed_net_records
        ]
        return updated, meander_report(
            [
                MeanderEdgeReport(
                    edge=short_edge,
                    requested_extra_length_um=25.5,
                    status="planned",
                    inserted_extra_length_um=25.5,
                    physical_inserted_extra_length_um=25.5,
                    planned_edges=(short_edge,),
                ),
                MeanderEdgeReport(
                    edge=long_edge,
                    requested_extra_length_um=25.0,
                    status="planned",
                    inserted_extra_length_um=25.0,
                    physical_inserted_extra_length_um=25.0,
                    planned_edges=(long_edge,),
                ),
            ],
            realized={short_edge: 25.5, long_edge: 25.0},
        )

    monkeypatch.setattr(
        routing_api,
        "minimum_four_bend_extra_length_um",
        lambda **_kwargs: 25.0,
    )
    monkeypatch.setattr(
        routing_api,
        "analyze_meander_insertion_for_requirements",
        _fake_meander_planner,
    )
    monkeypatch.setattr(
        routing_api,
        "realize_routed_net_records",
        lambda _layout, routed_net_records, **_kwargs: captured.update(
            realized_records=routed_net_records
        ),
    )

    result = route_rust.route_match_and_realize(
        Component(name="unrouted_lifted_pipeline_pass"),
        schematic,
        enable_path_length_matching=True,
        node_types={
            "gc_0": "input",
            "gc_1": "input",
            "mmi_0": "gate",
            "gc_2": "output",
            "gc_3": "output",
        },
        internal_delays_um={},
    )

    analysis_info = cast(dict[str, object], result.path_length_analysis_info)
    requirements_info = cast(list[dict[str, object]], result.meander_requirements_info)
    diagnostics = cast(
        list[dict[str, object]],
        analysis_info["matching_group_diagnostics"],
    )
    acceptance = cast(dict[str, object], analysis_info["path_length_acceptance"])
    realized_records = cast(list[RoutedNetRecord], captured["realized_records"])
    realized_by_edge = {
        RoutedEdgeKey(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
        ): record
        for record in realized_records
    }

    assert len(requirements_info) == 2
    assert analysis_info["minimum_insertable_extra_length_um"] == pytest.approx(25.0)
    assert diagnostics[0]["target_lift_um"] == pytest.approx(25.0)
    assert diagnostics[0]["max_physical_residual_um"] == pytest.approx(0.0)
    assert diagnostics[0]["max_disregarded_residual_um"] == pytest.approx(0.0)
    assert diagnostics[0]["within_tolerance"] is True
    assert acceptance["passed"] is True
    assert acceptance["failed_edge_count"] == 0
    assert diagnostics[0]["edges_requiring_meander"] == 2
    assert realized_by_edge[short_edge].meander_auto_plan is not None
    assert realized_by_edge[long_edge].meander_auto_plan is not None
    assert cast(dict[str, object], realized_by_edge[short_edge].meander_auto_plan)[
        "requested_extra_length_um"
    ] == pytest.approx(25.5)
    assert cast(dict[str, object], realized_by_edge[long_edge].meander_auto_plan)[
        "requested_extra_length_um"
    ] == pytest.approx(25.0)


def test_route_match_and_realize_rejects_unrealized_lifted_plm(monkeypatch):
    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=object(),
            total_length_um=99.5,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=object(),
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=object(),
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=object(),
            total_length_um=60.0,
        ),
    ]
    captured: dict[str, object] = {"realized": False}

    monkeypatch.setattr(
        routing_api,
        "route_nets_rust",
        lambda *args, **kwargs: (
            Component(name="failed_lifted_pipeline"),
            RustRouteDebugArtifacts(
                obstacle_svg=None,
                route_svgs=[],
                routed_edge_lengths_um={},
                routed_net_records=records,
                realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
                realization_bend_radius_cells=4,
            ),
        ),
    )

    class _ObstacleMap:
        blocked_cells: tuple[tuple[int, int], ...] = ()

    monkeypatch.setattr(
        routing_api,
        "build_static_obstacle_map",
        lambda *args, **kwargs: _ObstacleMap(),
    )

    def _fake_failed_planner(
        _routed_net_records: list[RoutedNetRecord],
        requirements: list[MissingLengthRequirement],
        **_kwargs: object,
    ) -> tuple[list[RoutedNetRecord], MeanderInsertionReport]:
        required_by_edge = {req.edge_key: req.missing_length_um for req in requirements}
        return records, meander_report(
            [
                MeanderEdgeReport(
                    edge=edge,
                    requested_extra_length_um=required_by_edge[edge],
                    status="no_candidate",
                    reason="synthetic planning failure",
                    unmatched_length_um=required_by_edge[edge],
                )
                for edge in (short_edge, long_edge)
            ]
        )

    monkeypatch.setattr(
        routing_api,
        "minimum_four_bend_extra_length_um",
        lambda **_kwargs: 25.0,
    )
    monkeypatch.setattr(
        routing_api,
        "analyze_meander_insertion_for_requirements",
        _fake_failed_planner,
    )
    monkeypatch.setattr(
        routing_api,
        "realize_routed_net_records",
        lambda *args, **kwargs: captured.update(realized=True),
    )

    with pytest.raises(RuntimeError, match="Path-length matching failed") as exc_info:
        route_rust.route_match_and_realize(
            Component(name="unrouted_lifted_pipeline_fail"),
            schematic,
            enable_path_length_matching=True,
            node_types={
                "gc_0": "input",
                "gc_1": "input",
                "mmi_0": "gate",
                "gc_2": "output",
                "gc_3": "output",
            },
            internal_delays_um={},
        )

    assert "gc0_to_mmi_in1" in str(exc_info.value)
    assert "no_candidate" in str(exc_info.value)
    assert captured["realized"] is False


def test_matching_group_diagnostics_reports_post_meander_residuals():
    schematic = path_length_schematic()
    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_short.net_name,
            source=edge_short.source,
            target=edge_short.target,
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )
    report = meander_report(
        [
            MeanderEdgeReport(
                edge=edge_short,
                requested_extra_length_um=20.0,
                status="planned",
                inserted_extra_length_um=20.0,
                planned_edges=(edge_short,),
            )
        ],
        realized={edge_short: 20.0},
    )

    groups = matching_group_diagnostics_to_info(result, report)

    assert len(groups) == 1
    assert groups[0]["within_tolerance"] is True
    assert groups[0]["max_accepted_unmatched_um"] == pytest.approx(0.0)
    assert groups[0]["max_physical_residual_um"] == pytest.approx(0.0)


def test_matching_group_diagnostics_tracks_disregarded_small_residual():
    schematic = path_length_schematic()
    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_short.net_name,
            source=edge_short.source,
            target=edge_short.target,
            route_obj=None,
            total_length_um=99.5,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )
    report = meander_report(
        [
            MeanderEdgeReport(
                edge=edge_short,
                requested_extra_length_um=0.5,
                status="below_minimum_bump",
            )
        ]
    )

    groups = matching_group_diagnostics_to_info(result, report)

    assert groups[0]["within_tolerance"] is True
    assert groups[0]["has_disregarded_residual"] is True
    assert groups[0]["max_accepted_unmatched_um"] == pytest.approx(0.0)
    assert groups[0]["max_physical_residual_um"] == pytest.approx(0.5)
    assert groups[0]["max_disregarded_residual_um"] == pytest.approx(0.5)


def test_path_length_acceptance_uses_physical_residual_not_accepted_unmatched():
    diagnostics = [
        {
            "node_name": "gate0",
            "node_type": "gate",
            "target_input_arrival_um": 100.0,
            "target_lift_um": 25.0,
            "max_physical_residual_um": 25.0,
            "incoming_edges": [
                {
                    "edge": {
                        "net_name": "n0",
                        "source": {"instance": "src0", "port": "o1"},
                        "target": {"instance": "gate0", "port": "i0"},
                    },
                    "adjusted_missing_length_um": 25.0,
                    "inserted_extra_length_um": 0.0,
                    "physical_residual_um": 25.0,
                    "accepted_unmatched_um": 0.0,
                    "disregarded_residual_um": 25.0,
                    "meander_status": "below_minimum_bump",
                }
            ],
        }
    ]

    summary = path_length_acceptance_summary(diagnostics)
    message = format_path_length_acceptance_failure(summary)

    assert summary["passed"] is False
    assert summary["failed_group_count"] == 1
    assert summary["failed_edge_count"] == 1
    assert summary["max_physical_residual_um"] == pytest.approx(25.0)
    assert "below_minimum_bump" in message
    assert "residual=25" in message


def test_path_length_acceptance_passes_exactly_realized_group():
    diagnostics = [
        {
            "node_name": "gate0",
            "node_type": "gate",
            "target_input_arrival_um": 125.0,
            "target_lift_um": 25.0,
            "max_physical_residual_um": 0.0,
            "incoming_edges": [
                {
                    "edge": {
                        "net_name": "n0",
                        "source": {"instance": "src0", "port": "o1"},
                        "target": {"instance": "gate0", "port": "i0"},
                    },
                    "adjusted_missing_length_um": 25.0,
                    "inserted_extra_length_um": 25.0,
                    "physical_residual_um": 0.0,
                    "accepted_unmatched_um": 0.0,
                    "disregarded_residual_um": 0.0,
                    "meander_status": "planned",
                }
            ],
        }
    ]

    summary = path_length_acceptance_summary(diagnostics)

    assert summary["passed"] is True
    assert summary["failed_group_count"] == 0
    assert summary["failed_edge_count"] == 0
    assert summary["max_physical_residual_um"] == pytest.approx(0.0)


def test_output_matching_diagnostics_fail_unrealized_output_requirement():
    edge_info = {
        "net_name": "out_short",
        "source": {"instance": "mmi0", "port": "o3"},
        "target": {"instance": "gc_out0", "port": "o1"},
    }
    output_info = {
        "enabled": True,
        "target_output_arrival_um": 100.0,
        "output_count": 2,
        "outputs": [
            {
                "node_name": "gc_out0",
                "arrival_um": 80.0,
                "missing_length_um": 20.0,
                "incoming_count": 1,
                "status": "requires_delay",
                "edge": edge_info,
            },
            {
                "node_name": "gc_out1",
                "arrival_um": 100.0,
                "missing_length_um": 0.0,
                "incoming_count": 1,
                "status": "not_required",
            },
        ],
    }
    report = meander_report(
        [
            MeanderEdgeReport(
                edge=RoutedEdgeKey(
                    net_name="out_short",
                    source=PortRef(instance="mmi0", port="o3"),
                    target=PortRef(instance="gc_out0", port="o1"),
                ),
                requested_extra_length_um=20.0,
                status="no_candidate",
                unmatched_length_um=20.0,
            )
        ]
    )

    diagnostics = output_matching_diagnostics_to_info(output_info, report)
    summary = path_length_acceptance_summary(diagnostics)

    assert diagnostics[0]["node_name"] == "output_arrivals"
    assert diagnostics[0]["max_physical_residual_um"] == pytest.approx(20.0)
    assert summary["passed"] is False
    assert summary["failed_edge_count"] == 1


def _build_two_stage_schematic_for_convergence() -> _SchematicLike:
    class _Bundle:
        def __init__(self, links: dict[str, str]):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {
                "input_a": object(),
                "input_b": object(),
                "gate_x": object(),
                "gate_z": object(),
            }
            self.routes = {
                "a_to_x": _Bundle({"input_a,o1": "gate_x,i0"}),
                "b_to_z": _Bundle({"input_b,o1": "gate_z,i1"}),
                "x_to_z": _Bundle({"gate_x,o1": "gate_z,i0"}),
            }

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    return _Schematic()


def test_missing_length_analysis_uses_internal_delay_at_output_node():
    schematic = _build_two_stage_schematic_for_convergence()
    records = [
        RoutedNetRecord(
            net_name="a_to_x",
            source=PortRef(instance="input_a", port="o1"),
            target=PortRef(instance="gate_x", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="b_to_z",
            source=PortRef(instance="input_b", port="o1"),
            target=PortRef(instance="gate_z", port="i1"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="x_to_z",
            source=PortRef(instance="gate_x", port="o1"),
            target=PortRef(instance="gate_z", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
    ]

    node_types = {
        "input_a": "input",
        "input_b": "input",
        "gate_x": "gate",
        "gate_z": "gate",
    }
    internal_delays = {"gate_x": 100.0}

    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=node_types,
        internal_delays_um=internal_delays,
    )

    edge_direct = RoutedEdgeKey(
        net_name="b_to_z",
        source=PortRef(instance="input_b", port="o1"),
        target=PortRef(instance="gate_z", port="i1"),
    )
    edge_via_x = RoutedEdgeKey(
        net_name="x_to_z",
        source=PortRef(instance="gate_x", port="o1"),
        target=PortRef(instance="gate_z", port="i0"),
    )

    assert result.node_arrival_input_um["gate_x"] == 10.0
    assert result.node_arrival_output_um["gate_x"] == 110.0
    assert result.node_arrival_input_um["gate_z"] == 120.0
    assert result.node_arrival_output_um["gate_z"] == 120.0
    assert result.edge_missing_lengths_um[edge_direct] == 110.0
    assert result.edge_missing_lengths_um[edge_via_x] == 0.0
    assert any(req.edge_key == edge_direct for req in result.requirements)
    assert any(req.edge_key == edge_via_x for req in result.requirements) is False
    timing = result.node_timings["gate_z"]
    assert timing.input_arrival_um == 120.0
    assert timing.output_arrival_um == 120.0
    assert timing.internal_delay_um == 0.0
    assert [timing_edge.edge_key for timing_edge in timing.incoming_edges] == [
        edge_direct,
        edge_via_x,
    ]


def test_missing_length_analysis_without_internal_delay_reduces_to_edge_length_sum():
    schematic = _build_two_stage_schematic_for_convergence()
    records = [
        RoutedNetRecord(
            net_name="a_to_x",
            source=PortRef(instance="input_a", port="o1"),
            target=PortRef(instance="gate_x", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="b_to_z",
            source=PortRef(instance="input_b", port="o1"),
            target=PortRef(instance="gate_z", port="i1"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="x_to_z",
            source=PortRef(instance="gate_x", port="o1"),
            target=PortRef(instance="gate_z", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
    ]

    node_types = {
        "input_a": "input",
        "input_b": "input",
        "gate_x": "gate",
        "gate_z": "gate",
    }

    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=node_types,
        internal_delays_um={},
    )

    edge_direct = RoutedEdgeKey(
        net_name="b_to_z",
        source=PortRef(instance="input_b", port="o1"),
        target=PortRef(instance="gate_z", port="i1"),
    )
    assert result.node_arrival_input_um["gate_z"] == 20.0
    assert result.edge_missing_lengths_um[edge_direct] == 10.0


def _build_schematic_from_links(
    instances: list[str],
    links_by_net: dict[str, dict[str, str]],
) -> _SchematicLike:
    class _Bundle:
        def __init__(self, links: dict[str, str]):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {name: object() for name in instances}
            self.routes = {net_name: _Bundle(links) for net_name, links in links_by_net.items()}

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    return _Schematic()


def test_output_matching_requirements_align_one_input_outputs_after_existing_delays():
    schematic = _build_schematic_from_links(
        ["src0", "src1", "out0", "out1"],
        {
            "n0": {"src0,o1": "out0,o1"},
            "n1": {"src1,o1": "out1,o1"},
        },
    )
    edge_short = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_long = RoutedEdgeKey(
        net_name="n1",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="out1", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_short.net_name,
            source=edge_short.source,
            target=edge_short.target,
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name=edge_long.net_name,
            source=edge_long.source,
            target=edge_long.target,
            route_obj=None,
            total_length_um=100.0,
        ),
    ]

    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={"src0": "input", "src1": "input", "out0": "output", "out1": "output"},
        internal_delays_um={},
    )
    requirements, info = compute_output_matching_requirements(
        analysis,
        existing_requirements=[
            MissingLengthRequirement(edge_key=edge_short, missing_length_um=5.0)
        ],
    )

    assert requirements == [MissingLengthRequirement(edge_key=edge_short, missing_length_um=15.0)]
    assert info["target_output_arrival_um"] == pytest.approx(100.0)
    assert info["output_count"] == 2


def test_output_matching_by_depth_aligns_only_outputs_of_the_same_depth():
    """The beam splitter's shape: two outputs tapped behind one gate, two behind two
    gates; with scope ``depth`` each pair is aligned among itself and the pairs
    stay apart, with scope ``all`` every output rises to the latest."""

    schematic = _build_schematic_from_links(
        ["src", "g1", "g2", "tap0", "tap1", "out0", "out1"],
        {
            "feed": {"src,o1": "g1,o1"},
            "t0": {"g1,o2": "tap0,o1"},
            "t1": {"g1,o3": "tap1,o1"},
            "link": {"g1,o4": "g2,o1"},
            "e0": {"g2,o2": "out0,o1"},
            "e1": {"g2,o3": "out1,o1"},
        },
    )
    lengths = {"feed": 10.0, "t0": 100.0, "t1": 130.0, "link": 500.0, "e0": 50.0, "e1": 80.0}

    def key(net: str) -> RoutedEdgeKey:
        ((p1, p2),) = schematic.netlist.routes[net].links.items()
        a, pa = p1.split(",")
        b, pb = p2.split(",")
        return RoutedEdgeKey(net_name=net, source=PortRef(a, pa), target=PortRef(b, pb))

    records = [
        RoutedNetRecord(
            net_name=net,
            source=key(net).source,
            target=key(net).target,
            route_obj=None,
            total_length_um=length,
        )
        for net, length in lengths.items()
    ]
    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src": "input",
            "tap0": "output",
            "tap1": "output",
            "out0": "output",
            "out1": "output",
        },
        internal_delays_um={},
    )
    by_depth, info = compute_output_matching_requirements(analysis, scope="depth")
    assert {req.edge_key.net_name: req.missing_length_um for req in by_depth} == {
        "t0": pytest.approx(30.0),
        "e0": pytest.approx(30.0),
    }
    assert info["scope"] == "depth"
    assert info["group_target_arrivals_um"] == {
        "2": pytest.approx(140.0),
        "3": pytest.approx(590.0),
    }
    everything, info_all = compute_output_matching_requirements(analysis, scope="all")
    assert {req.edge_key.net_name: round(req.missing_length_um, 1) for req in everything} == {
        "t0": 480.0,
        "t1": 450.0,
        "e0": 30.0,
    }
    assert info_all["target_output_arrival_um"] == pytest.approx(590.0)
    with pytest.raises(ValueError):
        compute_output_matching_requirements(analysis, scope="pairs")


def test_merge_missing_length_requirements_adds_independent_requirements_per_edge():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )

    merged = merge_missing_length_requirements(
        [MissingLengthRequirement(edge_key=edge, missing_length_um=20.0)],
        [MissingLengthRequirement(edge_key=edge, missing_length_um=5.0)],
    )

    assert merged == [MissingLengthRequirement(edge_key=edge, missing_length_um=25.0)]


def test_delay_candidates_include_transparent_heater_upstream_edge():
    schematic = _build_schematic_from_links(
        ["src0", "src1", "heater0", "mmi0"],
        {
            "src_to_heater": {"src0,o1": "heater0,o1"},
            "heater_to_mmi": {"heater0,o2": "mmi0,i0"},
            "src_to_mmi": {"src1,o1": "mmi0,i1"},
        },
    )
    edge_upstream = RoutedEdgeKey(
        net_name="src_to_heater",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="heater0", port="o1"),
    )
    edge_via_heater = RoutedEdgeKey(
        net_name="heater_to_mmi",
        source=PortRef(instance="heater0", port="o2"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_direct = RoutedEdgeKey(
        net_name="src_to_mmi",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_upstream.net_name,
            source=edge_upstream.source,
            target=edge_upstream.target,
            route_obj=None,
            total_length_um=50.0,
        ),
        RoutedNetRecord(
            net_name=edge_via_heater.net_name,
            source=edge_via_heater.source,
            target=edge_via_heater.target,
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name=edge_direct.net_name,
            source=edge_direct.source,
            target=edge_direct.target,
            route_obj=None,
            total_length_um=100.0,
        ),
    ]

    analysis, requirements = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src0": "input",
            "src1": "input",
            "heater0": "gate",
            "mmi0": "gate",
        },
        internal_delays_um={},
    )
    candidates = build_requirement_delay_candidates(analysis, requirements)

    assert requirements == [
        MissingLengthRequirement(edge_key=edge_via_heater, missing_length_um=40.0)
    ]
    assert [candidate.reason for candidate in candidates[edge_via_heater]] == [
        "direct_edge",
        "transparent_serial_upstream",
    ]
    assert candidates[edge_via_heater][1].edge_keys == (edge_upstream,)


def test_delay_candidates_include_common_mode_bundle_only_for_shared_deficit():
    schematic = _build_schematic_from_links(
        ["src0", "src1", "mmi0", "out0", "out1"],
        {
            "src0_to_mmi": {"src0,o1": "mmi0,i0"},
            "src1_to_mmi": {"src1,o1": "mmi0,i1"},
            "mmi_to_out0": {"mmi0,o0": "out0,o1"},
            "mmi_to_out1": {"mmi0,o1": "out1,o1"},
        },
    )
    edge_in0 = RoutedEdgeKey(
        net_name="src0_to_mmi",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_in1 = RoutedEdgeKey(
        net_name="src1_to_mmi",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    edge_out0 = RoutedEdgeKey(
        net_name="mmi_to_out0",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_out1 = RoutedEdgeKey(
        net_name="mmi_to_out1",
        source=PortRef(instance="mmi0", port="o1"),
        target=PortRef(instance="out1", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_in0.net_name,
            source=edge_in0.source,
            target=edge_in0.target,
            route_obj=None,
            total_length_um=50.0,
        ),
        RoutedNetRecord(
            net_name=edge_in1.net_name,
            source=edge_in1.source,
            target=edge_in1.target,
            route_obj=None,
            total_length_um=50.0,
        ),
        RoutedNetRecord(
            net_name=edge_out0.net_name,
            source=edge_out0.source,
            target=edge_out0.target,
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name=edge_out1.net_name,
            source=edge_out1.source,
            target=edge_out1.target,
            route_obj=None,
            total_length_um=10.0,
        ),
    ]

    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src0": "input",
            "src1": "input",
            "mmi0": "gate",
            "out0": "output",
            "out1": "output",
        },
        internal_delays_um={},
    )
    req0 = MissingLengthRequirement(edge_key=edge_out0, missing_length_um=30.0)
    req1 = MissingLengthRequirement(edge_key=edge_out1, missing_length_um=50.0)
    candidates = build_requirement_delay_candidates(analysis, [req0, req1])

    assert [candidate.reason for candidate in candidates[edge_out0]] == [
        "common_mode_upstream_bundle",
        "direct_edge",
    ]
    assert candidates[edge_out0][0].edge_keys == (edge_in0, edge_in1)
    assert candidates[edge_out0][0].affected_requirement_edge_keys == (
        edge_out0,
        edge_out1,
    )
    assert [candidate.reason for candidate in candidates[edge_out1]] == ["direct_edge"]


def test_delay_candidates_recursively_push_common_delay_through_heater_and_mmi():
    schematic = _build_schematic_from_links(
        ["src0", "src1", "mmi0", "heater0", "mmi1", "out0", "out1"],
        {
            "src0_to_mmi0": {"src0,o1": "mmi0,i0"},
            "src1_to_mmi0": {"src1,o1": "mmi0,i1"},
            "mmi0_upper_to_heater": {"mmi0,o0": "heater0,o1"},
            "heater_to_mmi1": {"heater0,o2": "mmi1,i0"},
            "mmi0_lower_to_mmi1": {"mmi0,o1": "mmi1,i1"},
            "mmi1_to_out0": {"mmi1,o0": "out0,o1"},
            "mmi1_to_out1": {"mmi1,o1": "out1,o1"},
        },
    )
    edge_in0 = RoutedEdgeKey(
        net_name="src0_to_mmi0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_in1 = RoutedEdgeKey(
        net_name="src1_to_mmi0",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    edge_mmi0_upper = RoutedEdgeKey(
        net_name="mmi0_upper_to_heater",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="heater0", port="o1"),
    )
    edge_heater = RoutedEdgeKey(
        net_name="heater_to_mmi1",
        source=PortRef(instance="heater0", port="o2"),
        target=PortRef(instance="mmi1", port="i0"),
    )
    edge_mmi0_lower = RoutedEdgeKey(
        net_name="mmi0_lower_to_mmi1",
        source=PortRef(instance="mmi0", port="o1"),
        target=PortRef(instance="mmi1", port="i1"),
    )
    edge_out0 = RoutedEdgeKey(
        net_name="mmi1_to_out0",
        source=PortRef(instance="mmi1", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_out1 = RoutedEdgeKey(
        net_name="mmi1_to_out1",
        source=PortRef(instance="mmi1", port="o1"),
        target=PortRef(instance="out1", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_in0.net_name,
            source=edge_in0.source,
            target=edge_in0.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_in1.net_name,
            source=edge_in1.source,
            target=edge_in1.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_mmi0_upper.net_name,
            source=edge_mmi0_upper.source,
            target=edge_mmi0_upper.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_heater.net_name,
            source=edge_heater.source,
            target=edge_heater.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_mmi0_lower.net_name,
            source=edge_mmi0_lower.source,
            target=edge_mmi0_lower.target,
            route_obj=None,
            total_length_um=40.0,
        ),
        RoutedNetRecord(
            net_name=edge_out0.net_name,
            source=edge_out0.source,
            target=edge_out0.target,
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name=edge_out1.net_name,
            source=edge_out1.source,
            target=edge_out1.target,
            route_obj=None,
            total_length_um=10.0,
        ),
    ]
    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src0": "input",
            "src1": "input",
            "mmi0": "gate",
            "heater0": "gate",
            "mmi1": "gate",
            "out0": "output",
            "out1": "output",
        },
        internal_delays_um={},
    )
    req0 = MissingLengthRequirement(edge_key=edge_out0, missing_length_um=50.0)
    req1 = MissingLengthRequirement(edge_key=edge_out1, missing_length_um=50.0)

    candidates = build_requirement_delay_candidates(analysis, [req0, req1])
    candidate_edges_by_reason = {
        candidate.reason: candidate.edge_keys for candidate in candidates[edge_out0]
    }

    assert candidate_edges_by_reason["common_mode_upstream_bundle"] == (
        edge_heater,
        edge_mmi0_lower,
    )
    assert candidate_edges_by_reason["recursive_transparent_serial_upstream"] == (
        edge_mmi0_upper,
        edge_mmi0_lower,
    )
    assert candidate_edges_by_reason["recursive_common_mode_upstream_bundle"] == (
        edge_in0,
        edge_in1,
    )


def test_main_flow_flag_enables_path_length_matching(monkeypatch):
    class _Bundle:
        def __init__(self, links):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {"src0": object(), "src1": object(), "gate0": object()}
            self.routes = {
                "n0": _Bundle({"src0,o1": "gate0,i0"}),
                "n1": _Bundle({"src1,o1": "gate0,i1"}),
            }

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    class _Layout:
        def __init__(self):
            self.name = "dummy_layout"
            self.bbox = (0, 0, 1, 1)
            self.info = {}

        def show(self):
            return None

    schematic = _Schematic()
    lengths = {
        RoutedEdgeKey(
            net_name="n0",
            source=PortRef(instance="src0", port="o1"),
            target=PortRef(instance="gate0", port="i0"),
        ): 80.0,
        RoutedEdgeKey(
            net_name="n1",
            source=PortRef(instance="src1", port="o1"),
            target=PortRef(instance="gate0", port="i1"),
        ): 100.0,
    }
    captured: dict[str, object] = {}

    monkeypatch.setattr(routing_flow, "load_benchmark", lambda _: schematic)
    monkeypatch.setattr(
        routing_flow_optical,
        "load_benchmark_metadata",
        lambda *args, **kwargs: {
            "node_types": {"src0": "input", "src1": "input", "gate0": "gate"},
            "internal_delays_um": {},
        },
    )
    monkeypatch.setattr(routing_flow, "layout_from_schematic", lambda _: _Layout())

    def _fake_route_match_and_realize(*args: object, **kwargs: object) -> RouteRustPipelineResult:
        captured.update(kwargs)
        return RouteRustPipelineResult(
            routed_layout=Component(name="dummy_routed"),
            debug_artifacts=RustRouteDebugArtifacts(
                obstacle_svg=None,
                route_svgs=[],
                routed_edge_lengths_um=lengths,
                routed_net_records=[
                    RoutedNetRecord(
                        net_name="n0",
                        source=PortRef(instance="src0", port="o1"),
                        target=PortRef(instance="gate0", port="i0"),
                        route_obj=object(),
                        total_length_um=80.0,
                    ),
                    RoutedNetRecord(
                        net_name="n1",
                        source=PortRef(instance="src1", port="o1"),
                        target=PortRef(instance="gate0", port="i1"),
                        route_obj=object(),
                        total_length_um=100.0,
                    ),
                ],
                # None (not a real grid spec) deliberately opts this test out of
                # routing_flow_verification.py's verify_and_attach_photonic_reports
                # gate (`if not routed_records or realization_grid_spec is None:
                # return`) -- that gate is unconditional final-geometry
                # verification unrelated to path-length matching, and this
                # test's fake RoutedNetRecords use bare `object()` route_obj
                # placeholders that a real PyO3 realization call cannot accept.
                # This test only cares about the PLM-specific layout.info
                # fields populated independently of that gate.
                realization_grid_spec=None,
            ),
            path_length_analysis_info={"requirements": [{}]},
            meander_requirements_info=[{"edge": {"net_name": "n0"}, "missing_length_um": 20.0}],
        )

    monkeypatch.setattr(
        routing_flow_optical,
        "route_match_and_realize",
        _fake_route_match_and_realize,
    )

    layout = routing_flow.run_routing_flow(
        "DUMMY",
        enable_path_length_matching=True,
        path_length_match_outputs=True,
    )

    assert captured["path_length_match_outputs"] is True
    assert "path_length_analysis" in layout.info
    assert "meander_requirements" in layout.info
    assert len(layout.info["meander_requirements"]) == 1


def test_main_flow_matching_uses_record_lengths(monkeypatch):
    class _Bundle:
        def __init__(self, links):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {"src0": object(), "src1": object(), "gate0": object()}
            self.routes = {
                "n0": _Bundle({"src0,o1": "gate0,i0"}),
                "n1": _Bundle({"src1,o1": "gate0,i1"}),
            }

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    class _Layout:
        def __init__(self):
            self.name = "dummy"
            self.bbox = (0, 0, 1, 1)
            self.info = {}

    monkeypatch.setattr(routing_flow, "load_benchmark", lambda _: _Schematic())
    monkeypatch.setattr(
        routing_flow_optical,
        "load_benchmark_metadata",
        lambda *args, **kwargs: {
            "node_types": {"src0": "input", "src1": "input", "gate0": "gate"},
            "internal_delays_um": {},
        },
    )
    monkeypatch.setattr(routing_flow, "layout_from_schematic", lambda _: _Layout())
    monkeypatch.setattr(
        routing_flow_optical,
        "route_match_and_realize",
        lambda *args, **kwargs: RouteRustPipelineResult(
            routed_layout=Component(name="dummy"),
            debug_artifacts=RustRouteDebugArtifacts(
                obstacle_svg=None,
                route_svgs=[],
                routed_edge_lengths_um={},
                routed_net_records=[],
                realization_grid_spec=(10, 10, 0.5, 0.0, 0.0),
            ),
            path_length_analysis_info={"requirements": [{}]},
            meander_requirements_info=[{"edge": {"net_name": "n0"}, "missing_length_um": 20.0}],
        ),
    )

    layout = routing_flow.run_routing_flow("DUMMY", enable_path_length_matching=True)
    assert len(layout.info["meander_requirements"]) == 1
    assert layout.info["meander_requirements"][0]["edge"]["net_name"] == "n0"


def test_meander_planner_commits_bundle_candidate_atomically(monkeypatch):
    class _RouteObj:
        def __init__(self, name: str):
            self.name = name
            self.cells = [(1, 1), (2, 2)]

    def _centerline(route_obj: _RouteObj) -> list[tuple[float, float]]:
        y = 10.0 if route_obj.name == "a" else 20.0
        return [(0.0, y), (100.0, y)]

    def _plan_requests(
        _candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[list[float]],
        **_kwargs: object,
    ) -> dict[str, object]:
        plans = []
        for edge_index, requested in enumerate(candidate_requested_extra_lengths_um[0]):
            offset = 10 if edge_index == 0 else 20
            plans.append(planned_meander(requested, (offset, offset, offset, offset)))
        return planned_result(plans, planning_mode="rust_registered_candidate")

    def _plan_final(
        geometry_indices: list[int],
        requested_extra_lengths_um: list[float],
        _min_insertable_extra_length_um: float,
        **_kwargs: object,
    ) -> dict[str, object]:
        plans = []
        plan_input_indices = []
        for input_index, (_geometry_index, requested) in enumerate(
            zip(geometry_indices, requested_extra_lengths_um, strict=False)
        ):
            offset = 10 if input_index == 0 else 20
            plans.append(planned_meander(requested, (offset, offset, offset, offset)))
            plan_input_indices.append(input_index)
        return planned_result(
            plans, planning_mode="rust_registered_sequence", plan_input_indices=plan_input_indices
        )

    backend = fake_backend(
        centerline=_centerline, plan_requests=_plan_requests, plan_final=_plan_final
    )
    monkeypatch.setattr(routing_api, "_load_rust_backend", lambda: backend)

    requirement_edge = RoutedEdgeKey(
        net_name="mmi_to_out",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_a = RoutedEdgeKey(
        net_name="src_a_to_mmi",
        source=PortRef(instance="src_a", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_b = RoutedEdgeKey(
        net_name="src_b_to_mmi",
        source=PortRef(instance="src_b", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_a.net_name,
            source=edge_a.source,
            target=edge_a.target,
            route_obj=_RouteObj("a"),
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name=edge_b.net_name,
            source=edge_b.source,
            target=edge_b.target,
            route_obj=_RouteObj("b"),
            total_length_um=100.0,
        ),
    ]
    req = MissingLengthRequirement(
        edge_key=requirement_edge,
        missing_length_um=32.0,
    )

    updated, report = analyze_meander_insertion_for_requirements(
        records,
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=2.0,
            auto_meander_endpoint_inset_um=4.0,
        ),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
        requirement_delay_candidates={
            requirement_edge: [
                DelayInsertionCandidate(
                    requirement_edge_key=requirement_edge,
                    edge_keys=(edge_a, edge_b),
                    extra_length_um=32.0,
                    reason="common_mode_upstream_bundle",
                )
            ]
        },
        debug_meanders=True,
    )

    updated_by_edge = {
        RoutedEdgeKey(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
        ): record
        for record in updated
    }
    results = report.results
    assert results[0].status == "planned"
    assert results[0].candidate_reason == "common_mode_upstream_bundle"
    assert results[0].planned_edges == (edge_a, edge_b)
    assert results[0].inserted_extra_length_um == pytest.approx(32.0)
    assert results[0].physical_inserted_extra_length_um == pytest.approx(64.0)
    assert report.realized_extra_by_edge == {
        edge_a: pytest.approx(32.0),
        edge_b: pytest.approx(32.0),
    }
    assert report.profile is not None
    assert report.profile.planner_calls == 4
    assert report.profile.final_planner_calls == 2
    assert updated_by_edge[edge_a].meander_auto_plan is not None
    assert updated_by_edge[edge_b].meander_auto_plan is not None


def test_meander_planner_combines_reused_physical_edge_requirements(monkeypatch):
    class _RouteObj:
        def __init__(self) -> None:
            self.cells = [(1, 1), (2, 2)]

    planned_requests: list[float] = []

    def _plan_requests(
        _candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[list[float]],
        **_kwargs: object,
    ) -> dict[str, object]:
        requested = float(candidate_requested_extra_lengths_um[0][0])
        planned_requests.append(requested)
        plan = planned_meander(requested, (10, 10, len(planned_requests), len(planned_requests)))
        return planned_result([plan], planning_mode="rust_registered_candidate")

    def _plan_final(
        _geometry_indices: list[int],
        requested_extra_lengths_um: list[float],
        _min_insertable_extra_length_um: float,
        **_kwargs: object,
    ) -> dict[str, object]:
        requested = float(requested_extra_lengths_um[0])
        planned_requests.append(requested)
        plan = planned_meander(requested, (10, 10, len(planned_requests), len(planned_requests)))
        return planned_result(
            [plan], planning_mode="rust_registered_sequence", plan_input_indices=[0]
        )

    backend = fake_backend(
        centerline=lambda _route_obj: [(0.0, 10.0), (200.0, 10.0)],
        plan_requests=_plan_requests,
        plan_final=_plan_final,
    )
    monkeypatch.setattr(routing_api, "_load_rust_backend", lambda: backend)

    physical_edge = RoutedEdgeKey(
        net_name="src_to_gate",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="gate", port="i0"),
    )
    output_edge = RoutedEdgeKey(
        net_name="gate_to_out",
        source=PortRef(instance="gate", port="o1"),
        target=PortRef(instance="out", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=physical_edge.net_name,
            source=physical_edge.source,
            target=physical_edge.target,
            route_obj=_RouteObj(),
            total_length_um=100.0,
        )
    ]
    requirements = [
        MissingLengthRequirement(edge_key=physical_edge, missing_length_um=100.0),
        MissingLengthRequirement(edge_key=output_edge, missing_length_um=25.0),
    ]

    updated, report = analyze_meander_insertion_for_requirements(
        records,
        requirements,
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=2.0,
            auto_meander_endpoint_inset_um=4.0,
        ),
        realization_grid_spec=(300, 300, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
        requirement_delay_candidates={
            output_edge: [
                DelayInsertionCandidate(
                    requirement_edge_key=output_edge,
                    edge_keys=(physical_edge,),
                    extra_length_um=25.0,
                    reason="recursive_transparent_serial_upstream",
                    affected_requirement_edge_keys=(output_edge,),
                )
            ]
        },
    )

    results = report.results
    final_plan = cast(dict[str, object], updated[0].meander_auto_plan)

    # the second requirement asks the physical edge for its new total, 125
    assert planned_requests == pytest.approx([100.0, 125.0, 125.0])
    assert results[0].inserted_extra_length_um == pytest.approx(100.0)
    assert results[1].inserted_extra_length_um == pytest.approx(25.0)
    assert results[1].planner_requested_extra_length_um == pytest.approx(125.0)
    assert results[1].existing_physical_extra_length_um == pytest.approx(100.0)
    assert results[1].physical_inserted_extra_length_um == pytest.approx(125.0)
    assert results[1].planned_edges == (physical_edge,)
    assert report.realized_extra_by_edge == {physical_edge: pytest.approx(125.0)}
    assert final_plan["requested_extra_length_um"] == pytest.approx(125.0)


def test_meander_planner_rejects_partial_bundle_candidate(monkeypatch):
    class _RouteObj:
        def __init__(self, name: str):
            self.name = name
            self.cells = [(1, 1), (2, 2)]

    def _centerline(route_obj: _RouteObj) -> list[tuple[float, float]]:
        y = 10.0 if route_obj.name == "a" else 20.0
        return [(0.0, y), (100.0, y)]

    def _plan_requests(
        candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[list[float]],
        **_kwargs: object,
    ) -> dict[str, object]:
        requested = float(candidate_requested_extra_lengths_um[0][0])
        # the first edge planned, the second did not: no bundle
        return {
            "status": "no_candidate",
            "planning_mode": "none",
            "split_part_count": None,
            "selected_candidate_index": None,
            "plans": [],
            "candidate_results": [
                {
                    "candidate_index": 0,
                    "status": "no_candidate",
                    "reason": f"no exact meander candidate for {requested}",
                    "failed_edge_index": len(candidate_geometry_indices[0]) - 1,
                    "plans": [{"inserted_extra_length_um": requested}],
                }
            ],
            "endpoint_inset_um": 4.0,
            "endpoint_insets_um": [4.0],
            "box_depths_um": [10.0],
        }

    backend = fake_backend(centerline=_centerline, plan_requests=_plan_requests)
    monkeypatch.setattr(routing_api, "_load_rust_backend", lambda: backend)

    requirement_edge = RoutedEdgeKey(
        net_name="mmi_to_out",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_a = RoutedEdgeKey(
        net_name="src_a_to_mmi",
        source=PortRef(instance="src_a", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_b = RoutedEdgeKey(
        net_name="src_b_to_mmi",
        source=PortRef(instance="src_b", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_a.net_name,
            source=edge_a.source,
            target=edge_a.target,
            route_obj=_RouteObj("a"),
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name=edge_b.net_name,
            source=edge_b.source,
            target=edge_b.target,
            route_obj=_RouteObj("b"),
            total_length_um=100.0,
        ),
    ]
    req = MissingLengthRequirement(
        edge_key=requirement_edge,
        missing_length_um=32.0,
    )

    updated, report = analyze_meander_insertion_for_requirements(
        records,
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=2.0,
            auto_meander_endpoint_inset_um=4.0,
        ),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
        requirement_delay_candidates={
            requirement_edge: [
                DelayInsertionCandidate(
                    requirement_edge_key=requirement_edge,
                    edge_keys=(edge_a, edge_b),
                    extra_length_um=32.0,
                    reason="common_mode_upstream_bundle",
                )
            ]
        },
        debug_meanders=True,
    )

    results = report.results
    assert results[0].status == "no_candidate"
    assert results[0].inserted_extra_length_um == pytest.approx(0.0)
    assert results[0].unmatched_length_um == pytest.approx(32.0)
    assert results[0].planned_edges == ()
    assert report.realized_extra_by_edge == {}
    assert report.profile is not None
    assert report.profile.planner_calls == 2
    assert all(record.meander_auto_plan is None for record in updated)


def test_main_meander_report_uses_auto_multi_bump_path():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    route_obj = real_route_obj(10, 10, 60, 10)
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=route_obj,
        total_length_um=30.0,
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=32.0)

    updated, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=5.0,
            max_meander_height_um=40.0,
            auto_meander_endpoint_inset_um=0.0,
        ),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
    )

    assert updated[0].total_length_um == 30.0
    results = report.results
    assert len(results) == 1
    entry = results[0]
    assert entry.status == "planned"
    assert entry.plan is not None
    assert entry.plan["planning_mode"] == "fill_box_multi_bump"
    assert int(cast(int, entry.plan["bumps"])) >= 1
    assert entry.plan["effective_bend_radius_um"] == pytest.approx(4.0)
    assert entry.plan["primitive_bend_radius_um"] == pytest.approx(4.0)
    info = report.to_info_dict()
    assert info["results"][0]["bumps"] == entry.plan["bumps"]
    assert info["total_inserted_extra_length_um"] == pytest.approx(32.0)


def test_meander_report_builds_no_profile_without_debug_meanders():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=real_route_obj(10, 10, 60, 10),
        total_length_um=30.0,
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=32.0)
    kwargs: dict[str, Any] = {
        "config": MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=5.0,
            max_meander_height_um=40.0,
            auto_meander_endpoint_inset_um=0.0,
        ),
        "realization_grid_spec": (200, 200, 1.0, 0.0, 0.0),
        "allow_45_degree_turns": True,
        "bend_radius_cells": 4,
    }

    _, report = analyze_meander_insertion_for_requirements([record], [req], **kwargs)
    assert report.profile is None
    info = report.to_info_dict()
    assert "planner_calls" not in info and "setup_profile" not in info

    _, debug_report = analyze_meander_insertion_for_requirements(
        [record], [req], debug_meanders=True, **kwargs
    )
    assert debug_report.profile is not None
    assert debug_report.profile.planner_calls >= 2
    assert debug_report.profile.search_config["endpoint_insets_um"] == [0.0]
    assert len(debug_report.profile.requirement_attempts) == 1
    assert debug_report.to_info_dict()["planner_calls"] == debug_report.profile.planner_calls
    assert debug_report.results[0].inserted_extra_length_um == pytest.approx(
        report.results[0].inserted_extra_length_um
    )


def test_auto_meander_endpoint_inset_relaxes_when_radius_seven_needs_more_run():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    route_obj = real_route_obj(
        10,
        60,
        83,
        60,
        bend_radius_cells=7,
        allow_45_degree_turns=False,
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=route_obj,
        total_length_um=73.0,
    )
    # 100 um at r = 7 needs three U-turns (one would need 56 um of height,
    # the box allows 40), whose 63 um insertion width does not fit the
    # 73 um run once 7 um is inset at both ends -- until the inset relaxes.
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=100.0)

    _fixed_updated, fixed_report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            max_meander_height_um=40.0,
            auto_meander_endpoint_inset_um=7.0,
        ),
        realization_grid_spec=(256, 256, 1.0, 0.0, 0.0),
        allow_45_degree_turns=False,
        bend_radius_cells=7,
        debug_meanders=True,
    )
    fixed_results = fixed_report.results
    assert fixed_results[0].status == "no_candidate"
    assert fixed_report.profile is not None
    assert fixed_report.profile.search_config["endpoint_insets_um"] == [7.0]

    updated, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            max_meander_height_um=40.0,
            auto_meander_endpoint_inset_um=None,
        ),
        realization_grid_spec=(256, 256, 1.0, 0.0, 0.0),
        allow_45_degree_turns=False,
        bend_radius_cells=7,
        debug_meanders=True,
    )

    entry = report.results[0]
    assert report.profile is not None
    search_config = report.profile.search_config
    search_endpoint_insets = cast(list[float], search_config["endpoint_insets_um"])
    assert entry.status == "planned"
    assert entry.inserted_extra_length_um == pytest.approx(100.0)
    assert entry.plan is not None
    assert cast(float, entry.plan["endpoint_inset_um"]) < 7.0
    assert search_config["max_height_um"] == pytest.approx(40.0)
    assert search_config["endpoint_inset_policy"] == "adaptive"
    assert search_endpoint_insets == [7.0, 5.25, 3.5, 1.75, 0.0]
    assert entry.plan["visual_bumps"] == 2
    assert entry.plan["quarter_turns"] == 8
    assert updated[0].meander_auto_plan is not None


def test_registered_meander_geometry_requirement_matches_legacy_opened_cells():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for registered meander path test.")

    grid = rust_backend.GridSpec(240, 120, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=2,
        allow_45_degree_turns=False,
    )
    astar = rust_backend.AStarConfig(max_iterations=200_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    route_obj = router.route_single_net(
        rust_backend.State(20, 60, 0),
        rust_backend.State(180, 60, 0),
    )
    centerline = router.route_port_corrected_centerline(route_obj)
    inflated_route_cells = {
        (int(x) + dx, int(y) + dy)
        for x, y in route_obj.cells
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
    }
    planner = rust_backend.MeanderPlanner(router)
    indices, open_counts, unique_count = planner.register(
        [route_obj],
        base_static_cells=[],
        route_clearance_radius_cells=1,
        reset=False,
    )

    assert indices == [0]
    assert open_counts == [len(inflated_route_cells)]
    assert unique_count == len(inflated_route_cells)
    assert planner.open_cell_count(0) == len(inflated_route_cells)
    geometry_indices = planner.register_geometries(
        [centerline],
        [0],
        [21],
    )
    assert geometry_indices == [0]

    kwargs = {
        "requested_extra_length_um": 75.311,
        "box_depths_um": [40.0, 30.0, 24.0, 20.0],
        "min_bend_radius_um": None,
        "min_straight_um": 2.0,
        "max_bumps": 21,
        "max_meander_height_um": 40.0,
        "min_segment_length_um": 2.0,
        "endpoint_inset_um": 0.0,
        "clearance_radius_cells": 0,
        "side_policy": "both",
        "planning_mode": "fill_box_multi_bump",
    }
    registered_result = planner.plan_requests(
        [geometry_indices],
        [[kwargs["requested_extra_length_um"]]],
        min_bend_radius_um=kwargs["min_bend_radius_um"],
        min_straight_um=kwargs["min_straight_um"],
        max_meander_height_um=kwargs["max_meander_height_um"],
        min_segment_length_um=kwargs["min_segment_length_um"],
        auto_endpoint_inset_um=kwargs["endpoint_inset_um"],
        clearance_radius_cells=kwargs["clearance_radius_cells"],
        side_policy=kwargs["side_policy"],
        planning_mode=kwargs["planning_mode"],
    )
    legacy = planner.plan_centerline(
        centerline,
        opened_cells=sorted(inflated_route_cells),
        extra_blocked_cells=None,
        **kwargs,
    )
    assert registered_result["status"] == "planned"
    registered_plans = cast(list[dict[str, object]], registered_result["plans"])
    assert len(registered_plans) == 1
    registered = registered_plans[0]

    assert registered["inserted_extra_length_um"] == pytest.approx(
        legacy["inserted_extra_length_um"]
    )
    assert registered["selected_grid_rect"] == legacy["selected_grid_rect"]
    assert registered["bumps"] == legacy["bumps"]

    selected_rect = cast(tuple[int, int, int, int], registered["selected_grid_rect"])
    planner.reserve_rect(
        selected_rect[0],
        selected_rect[1],
        selected_rect[2],
        selected_rect[3],
    )
    shifted_result = planner.plan_requests(
        [geometry_indices],
        [[kwargs["requested_extra_length_um"]]],
        min_bend_radius_um=kwargs["min_bend_radius_um"],
        min_straight_um=kwargs["min_straight_um"],
        max_meander_height_um=kwargs["max_meander_height_um"],
        min_segment_length_um=kwargs["min_segment_length_um"],
        auto_endpoint_inset_um=kwargs["endpoint_inset_um"],
        clearance_radius_cells=kwargs["clearance_radius_cells"],
        side_policy=kwargs["side_policy"],
        planning_mode=kwargs["planning_mode"],
    )

    assert shifted_result["status"] == "planned"
    shifted_plans = cast(list[dict[str, object]], shifted_result["plans"])
    shifted = shifted_plans[0]
    assert shifted["selected_grid_rect"] != selected_rect

    planner.clear_reserved()
    # one candidate with the same geometry twice: the second plan avoids the
    # first one's box (the sequence reserves within the candidate)
    sequence_result = planner.plan_requests(
        [[geometry_indices[0], geometry_indices[0]]],
        [
            [
                kwargs["requested_extra_length_um"],
                kwargs["requested_extra_length_um"],
            ]
        ],
        min_bend_radius_um=kwargs["min_bend_radius_um"],
        min_straight_um=kwargs["min_straight_um"],
        max_meander_height_um=kwargs["max_meander_height_um"],
        min_segment_length_um=kwargs["min_segment_length_um"],
        auto_endpoint_inset_um=kwargs["endpoint_inset_um"],
        clearance_radius_cells=kwargs["clearance_radius_cells"],
        side_policy=kwargs["side_policy"],
        planning_mode=kwargs["planning_mode"],
    )
    assert sequence_result["status"] == "planned"
    assert sequence_result["planning_mode"] == "rust_registered_candidate"
    sequence_plans = cast(list[dict[str, object]], sequence_result["plans"])
    assert len(sequence_plans) == 2
    assert sequence_plans[0]["inserted_extra_length_um"] == pytest.approx(
        kwargs["requested_extra_length_um"]
    )
    assert sequence_plans[1]["inserted_extra_length_um"] == pytest.approx(
        kwargs["requested_extra_length_um"]
    )
    assert sequence_plans[0]["selected_grid_rect"] != sequence_plans[1]["selected_grid_rect"]


def test_meander_insertion_adapts_bump_cap_for_large_matching_request():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for adaptive meander test.")

    grid = rust_backend.GridSpec(1200, 100, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=2,
        allow_45_degree_turns=False,
    )
    astar = rust_backend.AStarConfig(max_iterations=200_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    route_obj = router.route_single_net(
        rust_backend.State(10, 10, 0),
        rust_backend.State(1051, 10, 0),
    )

    edge = RoutedEdgeKey(
        net_name="long_match",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=route_obj,
        total_length_um=float(route_obj.total_length_um),
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=1459.0)

    updated, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=2.0,
            max_meander_height_um=20.0,
        ),
        realization_grid_spec=(1200, 100, 1.0, 0.0, 0.0),
        allow_45_degree_turns=False,
        bend_radius_cells=2,
        debug_meanders=True,
    )

    results = report.results
    assert len(results) == 1
    entry = results[0]
    assert entry.status == "planned"
    assert entry.inserted_extra_length_um == pytest.approx(1459.0)
    assert entry.unmatched_length_um == pytest.approx(0.0)
    committed_plan = cast(dict[str, object], updated[0].meander_auto_plan)
    assert int(cast(int, committed_plan["max_bumps"])) == 259
    # 1459 um at r = 2 within 20 um of height: legs = ceil(1459 / (20 - 2r(4 - pi)))
    # = 80, i.e. 79 U-turns.
    assert entry.plan is not None
    assert int(cast(int, entry.plan["bumps"])) == 79
    assert entry.plan.get("effective_bend_radius_um") is not None
    assert report.profile is not None
    assert report.profile.planner_elapsed_s >= 0.0
    assert report.final_planning_mode == "rust_registered_sequence"


def test_meander_planning_requires_registered_rust_planner(monkeypatch):
    backend = fake_backend()
    monkeypatch.setattr(routing_api, "_load_rust_backend", lambda: backend)

    class _RouteObj:
        cells = [(1, 1), (2, 2), (3, 3)]

    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=_RouteObj(),
        total_length_um=30.0,
        opened_cells=((7, 8), (9, 10)),
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=32.0)

    updated, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(enabled=True, min_candidate_straight_length_um=5.0),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
        static_blocked_cells=[(1, 1)],
    )

    static_cells = set(backend.routers[0].static_cells)
    assert (1, 1) in static_cells
    assert (7, 8) not in static_cells
    assert (9, 10) not in static_cells
    assert updated[0].opened_cells == record.opened_cells
    results = report.results
    assert results[0].status == "no_candidate"
    assert "could not prepare candidate inputs" in results[0].reason
    assert report.final_planning_mode == "none"


def test_meander_commit_uses_rust_reserved_rect_without_python_cells():
    class _RouteObj:
        cells = [(1, 1), (2, 2)]
        compressed_waypoints = [(0, 0), (100, 0)]

    planner = FakePlanner(None)

    committed_edge = RoutedEdgeKey(
        net_name="committed",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    committed_record = RoutedNetRecord(
        net_name=committed_edge.net_name,
        source=committed_edge.source,
        target=committed_edge.target,
        route_obj=_RouteObj(),
        total_length_um=30.0,
    )
    context = _MeanderPlannerContext(
        planner=cast(Any, planner),
        by_edge={
            committed_edge: committed_record,
        },
        updated={
            committed_edge: committed_record,
        },
        registered_geometry_index_by_edge={},
        max_bumps_by_edge={},
        base_static_cells=set(),
        grid_size_um=1.0,
        bend_radius_um=4.0,
        profile=_ContextProfile(),
    )

    context.commit_planned_edge(
        selected_edge_key=committed_edge,
        record=committed_record,
        rr={
            "inserted_extra_length_um": 12.0,
            "selected_grid_rect": (3, 4, 5, 6),
            "box_depth_um": 10.0,
        },
        requested=12.0,
        max_bumps=1,
        search_config=search_config(),
        endpoint_inset_um=0.0,
    )

    assert planner.reserved_rects == [(3, 4, 5, 6)]
    assert not hasattr(context, "reserved_meander_cells")
    assert not hasattr(context, "pending_reserved_meander_rects")
    assert context.profile is not None
    assert set(context.profile.commit_profile) == {
        "blocked_recheck_s",
        "rust_reserved_rect_update_s",
        "planned_record_s",
    }
    committed = cast(dict[str, object], context.updated[committed_edge].meander_auto_plan)
    assert committed["requested_extra_length_um"] == pytest.approx(12.0)
    assert committed["selected_grid_rect"] == (3, 4, 5, 6)


def test_too_short_route_reports_no_meander_candidate():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    route_obj = real_route_obj(10, 10, 12, 10)
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=route_obj,
        total_length_um=2.0,
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=10.0)

    _, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(enabled=True, min_candidate_straight_length_um=10.0),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
    )

    results = report.results
    assert len(results) == 1
    assert results[0].status in {"below_minimum_bump", "no_candidate"}


def test_near_minimum_meander_request_normalizes_roundoff():
    minimum = 2.0 * 3.141592653589793 * 8.0

    assert _normalize_minimum_insertable_request(
        minimum - 1.0e-13,
        minimum_insertable_extra_um=minimum,
    ) == pytest.approx(minimum)

    assert _normalize_minimum_insertable_request(
        minimum - 1.0e-3,
        minimum_insertable_extra_um=minimum,
    ) == pytest.approx(minimum - 1.0e-3)


def test_minimum_insertable_uses_fill_box_meander_lower_bound():
    minimum = minimum_four_bend_extra_length_um(
        grid_size_um=1.0,
        bend_radius_cells=8,
    )

    # One U-turn at the minimum amplitude 2r + min_straight, two legs.
    assert minimum == pytest.approx(2.0 * (1.0 + (3.141592653589793 - 2.0) * 8.0))
    assert minimum < 46.75


def test_meander_search_config_uses_one_um_default_internal_straight():
    search = _meander_search_config(
        config=MeanderInsertionConfig(enabled=True, max_meander_height_um=80.0),
        bend_radius_um=10.0,
    )

    assert search.min_straight_um == pytest.approx(1.0)


def test_meander_search_config_includes_large_user_height():
    search = _meander_search_config(
        config=MeanderInsertionConfig(enabled=True, max_meander_height_um=1000.0),
        bend_radius_um=8.0,
    )

    assert search.box_depths_um[0] == pytest.approx(1000.0)
    assert 40.0 in search.box_depths_um


def test_matching_group_diagnostics_use_the_realized_placement_of_a_meander():
    """Bookkeeping by physical edge (2026-10-05): a meander booked on one
    incoming edge of the gate but drawn on the other (a delay-insertion
    candidate) shows up in ``realized_inserted_extra_length_um`` of the
    edge it was drawn on; the realized arrivals are walked over the graph,
    so the gate's realized spread reports the physical imbalance -- here
    the planned view is balanced (booked on the short edge) while the
    realized view is off by twice the meander, which the acceptance summary
    turns into a failure.
    """

    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    booked = MeanderEdgeReport(
        edge=short_edge,
        requested_extra_length_um=20.0,
        status="planned",
        inserted_extra_length_um=20.0,
    )
    # realized where booked: balanced in both views
    report = meander_report([booked], realized={short_edge: 20.0})
    groups = matching_group_diagnostics_to_info(result, report)
    assert len(groups) == 1
    by_net = {edge["edge"]["net_name"]: edge for edge in groups[0]["incoming_edges"]}
    assert by_net[short_edge.net_name]["realized_inserted_extra_length_um"] == pytest.approx(20.0)
    assert by_net[short_edge.net_name]["realized_length_um"] == pytest.approx(100.0)
    assert groups[0]["realized_arrival_spread_um"] == pytest.approx(0.0)
    assert groups[0]["realized_within_tolerance"] is True
    assert path_length_acceptance_summary(groups)["passed"] is True

    # realized on the other edge: the planned view still says balanced, the
    # realized view reports the imbalance and the acceptance fails on it
    report_misplaced = meander_report([booked], realized={long_edge: 20.0})
    groups = matching_group_diagnostics_to_info(result, report_misplaced)
    assert groups[0]["within_tolerance"] is True
    assert groups[0]["realized_arrival_spread_um"] == pytest.approx(40.0)
    assert groups[0]["realized_within_tolerance"] is False
    summary = path_length_acceptance_summary(groups)
    assert summary["passed"] is False
    assert summary["realized_failed_group_count"] == 1
    assert "realized bookkeeping" in format_path_length_acceptance_failure(summary)

    # the GDS meta info form carries the realized table per edge
    info = report_misplaced.to_info_dict()
    assert info["realized_extra_length_um_by_edge"] == [
        {
            "edge": {
                "net_name": long_edge.net_name,
                "source": {"instance": "gc_1", "port": "o1"},
                "target": {"instance": "mmi_0", "port": "o1"},
            },
            "extra_length_um": 20.0,
        }
    ]
    assert info["results"][0]["inserted_extra_length_um"] == pytest.approx(20.0)
