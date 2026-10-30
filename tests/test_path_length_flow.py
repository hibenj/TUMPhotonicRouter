"""Path-length matching through the flow, with the routing, planning and
realization stages monkeypatched: `route_match_and_realize` on a lifted
sub-bump group and on an unrealized one, and the flow flag that enables
matching."""

from __future__ import annotations

from dataclasses import replace
from typing import cast

import photonic_router.routing.route_rust as route_rust
import pytest
from gdsfactory.component import Component
from photonic_router import flow as routing_flow
from photonic_router import flow_optical
from photonic_router.plm.path_length_graph import (
    MissingLengthRequirement,
    PortRef,
    RoutedEdgeKey,
)
from photonic_router.records.route_rust_types import MeanderEdgeReport, MeanderInsertionReport
from photonic_router.routing import api as routing_api
from photonic_router.routing.route_rust import (
    RoutedNetRecord,
    RouteRustPipelineResult,
    RustRouteDebugArtifacts,
)

from tests.fixtures.path_length import (
    FakeLayout,
    meander_report,
    schematic_from_links,
)
from tests.fixtures.synthetic_layouts import (
    path_length_schematic,
)


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


def test_main_flow_flag_enables_path_length_matching(monkeypatch):
    schematic = schematic_from_links(
        ["src0", "src1", "gate0"],
        {"n0": {"src0,o1": "gate0,i0"}, "n1": {"src1,o1": "gate0,i1"}},
    )
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
        flow_optical,
        "load_benchmark_metadata",
        lambda *args, **kwargs: {
            "node_types": {"src0": "input", "src1": "input", "gate0": "gate"},
            "internal_delays_um": {},
        },
    )
    monkeypatch.setattr(routing_flow, "layout_from_schematic", lambda _: FakeLayout())

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
                # python/photonic_router/flow_verification.py's verify_and_attach_photonic_reports
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
        flow_optical,
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
    schematic = schematic_from_links(
        ["src0", "src1", "gate0"],
        {"n0": {"src0,o1": "gate0,i0"}, "n1": {"src1,o1": "gate0,i1"}},
    )

    monkeypatch.setattr(routing_flow, "load_benchmark", lambda _: schematic)
    monkeypatch.setattr(
        flow_optical,
        "load_benchmark_metadata",
        lambda *args, **kwargs: {
            "node_types": {"src0": "input", "src1": "input", "gate0": "gate"},
            "internal_delays_um": {},
        },
    )
    monkeypatch.setattr(routing_flow, "layout_from_schematic", lambda _: FakeLayout("dummy"))
    monkeypatch.setattr(
        flow_optical,
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
