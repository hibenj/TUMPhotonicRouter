"""One test per routing stage Protocol (`translation/routing/stages.py`).

Milestone 5 Slice 2c. Slice 2b declared the nine phases of `session.run()` as
`typing.Protocol`s; these tests exercise each phase on its own, on the smallest
synthetic scenario the existing session tests use (an empty layout, a dummy
netlist of three nets, a 30x20 one-micron grid from a stand-in obstacle builder),
with no benchmark and no schematic loading. Every phase gets the product of the
phases before it, so each test asserts on one phase's own product -- the frozen
dataclass it returns, or, for the two phases that return `None`, the state fields
it writes.

The synthetic scenario and the pipeline that runs the stages over it moved to
`tests/fixtures/synthetic_layouts.py` and `tests/fixtures/sessions.py` in
Milestone 6 Slice 2 (the `pipeline` fixture below comes from
`tests/conftest.py`); this file keeps only the per-phase assertions.

The last test is the conformance one: each phase function's signature is checked
against the `__call__` of its Protocol at run time, and the nine module-level
annotated assignments next to it are the same check for a type checker.
"""

from __future__ import annotations

import dataclasses
import inspect
import sys
from pathlib import Path
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from translation.routing import crossing_plan_stage as crossing_plan_stage_module
from translation.routing import dispatch as dispatch_module
from translation.routing import finalize as finalize_module
from translation.routing import handoff as handoff_module
from translation.routing import obstacle_context as obstacle_context_module
from translation.routing import realize as realize_module
from translation.routing import route_jobs as route_jobs_module
from translation.routing import router_setup as router_setup_module
from translation.routing import stages
from translation.routing import verify_repair as verify_repair_module
from translation.routing.crossing_plan_info import CrossingPlanInfo

from tests.fixtures.sessions import Pipeline, pipeline_for_test
from tests.fixtures.synthetic_layouts import GRID_HEIGHT, GRID_WIDTH, LINKS, ObstacleMapStandIn


def test_phase1_obstacle_context_builder_returns_the_map_and_the_grid(pipeline: Pipeline) -> None:
    context = pipeline.through(1)

    assert isinstance(context, stages.ObstacleContext)
    assert isinstance(context.obstacle_map, ObstacleMapStandIn)
    assert context.obstacle_svg is None  # no debug_dir in this session
    assert context.crossing_device_info == {
        "requested_half_size_cells": 0,
        "derived_from_component": False,
        "half_size_cells": 0,
    }
    assert pipeline.state.resolved_crossing_half_size_cells == 0
    # The grid the map defines is what the later phases measure against; the
    # cell counts themselves are phase 2's (see state.py).
    assert pipeline.state.grid is context.obstacle_map.grid
    assert (pipeline.state.grid_width, pipeline.state.grid_height) == (0, 0)


def test_phase2_router_setup_builds_the_kernel_router_into_the_state(pipeline: Pipeline) -> None:
    assert pipeline.through(2) is None  # phase 2's product is the state

    state = pipeline.state
    assert state.router is not None
    assert hasattr(state.router, "route_many_with_negotiated_repair_and_commit")
    assert state.primitive_cfg is not None
    assert state.astar_cfg is not None
    assert (state.grid_width, state.grid_height) == (GRID_WIDTH, GRID_HEIGHT)
    assert (state.origin_x_um, state.origin_y_um) == (0.0, 0.0)
    assert state.bend_radius_cells == 1  # bend_radius_um=1.0 over a 1 um grid
    assert state.raw_static_cells == set()  # the stand-in map blocks nothing
    # The realization grid spec is phase 5's, not this phase's (see state.py).
    assert state.realization_grid_spec is None


def test_phase3_route_job_builder_turns_the_netlist_into_jobs(pipeline: Pipeline) -> None:
    result = pipeline.through(3)

    assert isinstance(result, stages.RouteJobsResult)
    assert [job.net_name for job in result.route_jobs] == list(LINKS)
    assert [job.net_id for job in result.route_jobs] == [1, 2, 3]
    assert [(job.inst1, job.port1, job.inst2, job.port2) for job in result.route_jobs] == [
        ("left", "o1", "right0", "o1"),
        ("left", "o2", "right1", "o1"),
        ("left", "o3", "right2", "o1"),
    ]
    assert result.endpoint_port_specs_by_instance["left"] == {"left,o1", "left,o2", "left,o3"}
    assert result.endpoint_port_specs_by_instance["right0"] == {"right0,o1"}
    # Only the three-port instance is dense enough for a runway.
    assert set(result.dense_port_runway_length_by_spec) == {"left,o1", "left,o2", "left,o3"}


def test_phase4_crossing_planner_types_the_plan_and_passes_the_jobs_through(
    pipeline: Pipeline,
) -> None:
    jobs = pipeline.through(3)
    planned = pipeline.through(4)

    assert isinstance(planned, stages.PlannedJobs)
    assert planned.route_jobs == jobs.route_jobs
    assert set(planned.foreign_port_keepout_cells_by_instance) <= {
        "left",
        "right0",
        "right1",
        "right2",
    }

    # Slice 2c: the plan the phase writes is a typed record, not a bare dict.
    plan = pipeline.state.crossing_plan_info
    assert isinstance(plan, CrossingPlanInfo)
    assert plan.enabled is False  # crossings are off by default in this session
    assert plan.crossing_mode == pipeline.settings.crossing_mode
    assert plan.crossing_device == pipeline.through(1).crossing_device_info
    assert plan.bend_runout_cells_per_crossing == pipeline.state.bend_radius_cells
    assert plan.fanout_access_mode == pipeline.settings.fanout_access_mode_normalized
    assert plan.events == []
    # The mapping the artifacts carry has exactly the keys that were written.
    assert "realized_intersections" not in plan.to_dict()
    assert plan.to_dict()["crossing_mode"] == pipeline.settings.crossing_mode
    assert pipeline.state.normal_port_runway_cells


def test_phase5_static_handoff_orders_the_jobs_and_starts_the_search_clock(
    pipeline: Pipeline,
) -> None:
    planned = pipeline.through(4)
    final = pipeline.through(5)

    assert isinstance(final, stages.FinalJobs)
    assert sorted(job.net_id for job in final.route_jobs) == sorted(
        job.net_id for job in planned.route_jobs
    )
    assert final.astar_start_s > 0.0  # the search clock started
    assert pipeline.state.route_jobs_by_id.keys() == {1, 2, 3}
    # The realization grid this phase hands on is the grid phase 1 measured.
    assert pipeline.state.realization_grid_spec == (GRID_WIDTH, GRID_HEIGHT, 1.0, 0.0, 0.0)
    assert pipeline.state.repair_config is not None
    assert set(pipeline.state.batch_clearance_exempt_cells_by_id) == {1, 2, 3}


def test_phase6_kernel_dispatcher_leaves_the_routes_in_the_bookkeeping(
    pipeline: Pipeline,
) -> None:
    assert pipeline.through(6) is None  # phase 6's product is the state

    state = pipeline.state
    assert sorted(state.route_bookkeeping.records_by_id) == [1, 2, 3]
    assert state.deferred_count == 0
    assert state.total_expanded_states >= 0
    assert len(state.route_bookkeeping.ordered_records()) == 3


def test_phase7_result_finalizer_returns_one_record_per_net(pipeline: Pipeline) -> None:
    routed = pipeline.through(7)

    assert isinstance(routed, stages.RoutedRecords)
    assert [record.net_name for record in routed.routed_net_records] == list(LINKS)
    assert routed.astar_elapsed_s > 0.0
    for record in routed.routed_net_records:
        assert record.total_length_um > 0.0
        assert len(record.corrected_centerline_um) >= 2


def test_phase8_geometry_verifier_finds_no_illegal_crossing_on_parallel_nets(
    pipeline: Pipeline,
) -> None:
    routed = pipeline.through(7)
    verified = pipeline.through(8)

    assert isinstance(verified, stages.VerifiedRecords)
    assert verified.illegal_realized_crossings == []
    assert [record.net_name for record in verified.routed_net_records] == [
        record.net_name for record in routed.routed_net_records
    ]


def test_phase9_realizer_assembles_the_debug_artifacts_with_the_plan_as_a_mapping(
    pipeline: Pipeline,
) -> None:
    verified = pipeline.through(8)
    artifacts = pipeline.through(9)

    assert artifacts.realization_grid_spec == pipeline.state.realization_grid_spec
    assert [record.net_name for record in artifacts.routed_net_records] == [
        record.net_name for record in verified.routed_net_records
    ]
    assert artifacts.route_search_summary.route_count == 3
    # Slice 2c: the artifacts carry the plan as the plain mapping its consumers
    # (routing_flow_verification, the crossing report) still read.
    assert isinstance(artifacts.crossing_plan_info, dict)
    assert artifacts.crossing_plan_info == pipeline.state.crossing_plan_info.to_dict()
    # Realization happened on the session's layout, not on the unrouted one.
    assert pipeline.state.routed_layout.get_polygons()


# The nine (Protocol, phase function) pairs the conformance test below checks.
_STAGE_IMPLEMENTATIONS: tuple[tuple[str, type, Any], ...] = (
    (
        "ObstacleContextBuilder",
        stages.ObstacleContextBuilder,
        obstacle_context_module.build_static_obstacle_context,
    ),
    ("RouterSetup", stages.RouterSetup, router_setup_module.configure_router_and_grid),
    (
        "RouteJobBuilder",
        stages.RouteJobBuilder,
        route_jobs_module.build_route_jobs_and_fanout_clustering,
    ),
    (
        "CrossingPlanner",
        stages.CrossingPlanner,
        crossing_plan_stage_module.build_crossing_plan_and_port_footprints,
    ),
    ("StaticHandoff", stages.StaticHandoff, handoff_module.finalize_route_jobs_and_static_handoff),
    ("KernelDispatcher", stages.KernelDispatcher, dispatch_module.dispatch_native_routing),
    ("ResultFinalizer", stages.ResultFinalizer, finalize_module.finalize_routing_results),
    (
        "GeometryVerifier",
        stages.GeometryVerifier,
        verify_repair_module.repair_and_verify_final_geometry,
    ),
    ("Realizer", stages.Realizer, realize_module.realize_and_assemble_debug_artifacts),
)

# The same pairing as a typed assignment, for a type checker rather than pytest.
_OBSTACLE_CONTEXT_BUILDER: stages.ObstacleContextBuilder = (
    obstacle_context_module.build_static_obstacle_context
)
_ROUTER_SETUP: stages.RouterSetup = router_setup_module.configure_router_and_grid
_ROUTE_JOB_BUILDER: stages.RouteJobBuilder = (
    route_jobs_module.build_route_jobs_and_fanout_clustering
)
_CROSSING_PLANNER: stages.CrossingPlanner = (
    crossing_plan_stage_module.build_crossing_plan_and_port_footprints
)
_STATIC_HANDOFF: stages.StaticHandoff = handoff_module.finalize_route_jobs_and_static_handoff
_KERNEL_DISPATCHER: stages.KernelDispatcher = dispatch_module.dispatch_native_routing
_RESULT_FINALIZER: stages.ResultFinalizer = finalize_module.finalize_routing_results
_GEOMETRY_VERIFIER: stages.GeometryVerifier = verify_repair_module.repair_and_verify_final_geometry
_REALIZER: stages.Realizer = realize_module.realize_and_assemble_debug_artifacts


def _weighted_astar_pipeline(
    monkeypatch: pytest.MonkeyPatch, *, enable_crossings: bool
) -> Pipeline:
    """A phase-2 pipeline on Weighted A* (`min_heuristic_weight > 1.0`).

    `max_iterations` is deliberately above the cap, so the assertion is about
    the cap and not about `min()` picking the caller's smaller budget.
    """

    from photonic_router.config import RoutingConfig

    base = RoutingConfig.from_environment()
    config = dataclasses.replace(
        base, search=dataclasses.replace(base.search, min_heuristic_weight=2.0)
    )
    return pipeline_for_test(
        monkeypatch,
        config=config,
        max_iterations=200_000,
        enable_crossings=enable_crossings,
    )


def test_router_setup_caps_weighted_astar_iterations_when_crossings_are_off(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Restored in Milestone 8 Slice C: the Weighted-A* iteration cap.

    Slice B removed it with the collision crossing mode, but its condition was
    `not (enable_crossings and is_collision_mode(...))`, which with the modes
    gone is exactly `not enable_crossings` -- not "always false". 45-degree
    turns on, crossings off, Weighted A*: the cap applies.
    """

    pipeline = _weighted_astar_pipeline(monkeypatch, enable_crossings=False)
    assert pipeline.through(2) is None
    assert pipeline.settings.allow_45_degree_turns is True
    assert pipeline.settings.config.search.min_heuristic_weight == 2.0
    assert pipeline.state.astar_cfg.max_iterations == 50_000


def test_router_setup_leaves_weighted_astar_iterations_alone_when_crossings_are_on(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The other half of the folded condition: crossings on, no cap."""

    pipeline = _weighted_astar_pipeline(monkeypatch, enable_crossings=True)
    assert pipeline.through(2) is None
    assert pipeline.state.astar_cfg.max_iterations == 200_000


@pytest.mark.parametrize(
    ("name", "protocol", "implementation"),
    _STAGE_IMPLEMENTATIONS,
    ids=[name for name, _protocol, _implementation in _STAGE_IMPLEMENTATIONS],
)
def test_every_stage_function_satisfies_its_protocol(
    name: str, protocol: type, implementation: Any
) -> None:
    """The phase's parameters and product are the ones its Protocol declares."""

    declared = inspect.signature(protocol.__call__)
    actual = inspect.signature(implementation)
    declared_parameters = [
        parameter for key, parameter in declared.parameters.items() if key != "self"
    ]

    assert [parameter.name for parameter in declared_parameters] == [
        parameter.name for parameter in actual.parameters.values()
    ], name
    assert [parameter.kind for parameter in declared_parameters] == [
        parameter.kind for parameter in actual.parameters.values()
    ], name
    assert [str(parameter.annotation) for parameter in declared_parameters] == [
        str(parameter.annotation) for parameter in actual.parameters.values()
    ], name
    assert str(declared.return_annotation) == str(actual.return_annotation), name
