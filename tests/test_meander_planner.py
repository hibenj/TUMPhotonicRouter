"""The meander planner on the Python side, driven through the fake Rust
backend and `MeanderPlanner` of `tests/fixtures/path_length.py` and, where
the geometry matters, the real one: the planner context, the request vector
per requirement, bundle commits, the reports and the search policy."""

from __future__ import annotations

from typing import Any, cast

import pytest
from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
    MissingLengthRequirement,
    PortRef,
    RoutedEdgeKey,
)
from photonic_router.static_obstacle_builder import _load_rust_backend

from tests.fixtures.path_length import (
    FakePlanner,
    fake_backend,
    planned_meander,
    planned_result,
    real_route_obj,
    search_config,
)
from translation.route_rust import (
    MeanderInsertionConfig,
    RoutedNetRecord,
    analyze_meander_insertion_for_requirements,
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
from translation.routing import api as routing_api


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
