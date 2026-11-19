"""One test per routing stage Protocol (`python/photonic_router/routing/stages.py`).

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

import inspect
import sys
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from photonic_router.records.crossing_plan_info import CrossingPlanInfo
from photonic_router.routing import crossing_plan_stage as crossing_plan_stage_module
from photonic_router.routing import dispatch as dispatch_module
from photonic_router.routing import finalize as finalize_module
from photonic_router.routing import handoff as handoff_module
from photonic_router.routing import obstacle_context as obstacle_context_module
from photonic_router.routing import realize as realize_module
from photonic_router.routing import route_jobs as route_jobs_module
from photonic_router.routing import router_setup as router_setup_module
from photonic_router.routing import stages
from photonic_router.routing import verify_repair as verify_repair_module
from photonic_router.routing.products import KernelSetup, RawStaticGeometry

from tests.fixtures.sessions import Pipeline
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
    assert context.resolved_crossing_half_size_cells == 0
    # The grid the map defines is what the later phases measure against.
    assert context.grid is context.obstacle_map.grid
    assert context.debug_path is None and not context.diagnostics_enabled


def test_phase2_router_setup_returns_the_kernel_setup(pipeline: Pipeline) -> None:
    kernel = pipeline.through(2)

    assert isinstance(kernel, KernelSetup)
    state = pipeline.state
    assert state.kernel is kernel and state.obstacles is pipeline.through(1)
    # phase 5's records are still the empty ones
    assert state.static_cells.blocked_cells_before_port_reservations == set()
    assert state.kernel.router is not None
    assert hasattr(state.kernel.router, "route_many_with_negotiated_repair_and_commit")
    assert state.kernel.primitive_cfg is not None
    assert state.kernel.astar_cfg is not None
    assert (state.kernel.frame.grid_width, state.kernel.frame.grid_height) == (
        GRID_WIDTH,
        GRID_HEIGHT,
    )
    assert (state.kernel.frame.origin_x_um, state.kernel.frame.origin_y_um) == (0.0, 0.0)
    assert state.kernel.radii.bend_radius_cells == 1  # bend_radius_um=1.0 over a 1 um grid
    assert state.kernel.static.raw_static_cells == set()  # the stand-in map blocks nothing
    # The realization grid spec is phase 5's, not this phase's (see state.py).
    assert state.run.realization_grid_spec == (0, 0, 0.0, 0.0, 0.0)


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


def test_phase3_stores_one_frozen_record_per_step(pipeline: Pipeline) -> None:
    pipeline.through(3)
    state = pipeline.state

    assert set(state.endpoints.ports_by_spec) == {
        "left,o1",
        "left,o2",
        "left,o3",
        "right0,o1",
        "right1,o1",
        "right2,o1",
    }
    assert state.endpoints.source_specs_by_instance == {"left": {"left,o1", "left,o2", "left,o3"}}
    assert set(state.runways.source_length_by_spec) == {"left,o1", "left,o2", "left,o3"}
    # a record is assigned as a whole; its fields cannot be rebound afterwards
    with pytest.raises(FrozenInstanceError):
        state.fanout.anchor_net_ids = set()  # type: ignore[misc]


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
    assert plan.bend_runout_cells_per_crossing == pipeline.state.kernel.radii.bend_radius_cells
    assert plan.fanout_access_mode == pipeline.settings.fanout_access_mode_normalized
    assert plan.events == []
    # The mapping the artifacts carry has exactly the keys that were written.
    assert "realized_intersections" not in plan.to_dict()
    assert plan.to_dict()["crossing_mode"] == pipeline.settings.crossing_mode
    assert pipeline.state.ports.normal_runway_cells


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
    assert pipeline.state.run.route_jobs_by_id.keys() == {1, 2, 3}
    # The realization grid this phase hands on is the grid phase 1 measured.
    assert pipeline.state.run.realization_grid_spec == (GRID_WIDTH, GRID_HEIGHT, 1.0, 0.0, 0.0)
    assert pipeline.state.run.repair_config is not None
    assert set(pipeline.state.run.clearance_exempt_cells_by_id) == {1, 2, 3}


def test_phase6_kernel_dispatcher_leaves_the_routes_in_the_bookkeeping(
    pipeline: Pipeline,
) -> None:
    assert pipeline.through(6) is None  # phase 6's product is the state

    state = pipeline.state
    assert sorted(state.search.bookkeeping.records_by_id) == [1, 2, 3]
    assert state.search.total_expanded_states >= 0
    assert len(state.search.bookkeeping.ordered_records()) == 3


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

    assert artifacts.realization_grid_spec == pipeline.state.run.realization_grid_spec
    assert [record.net_name for record in artifacts.routed_net_records] == [
        record.net_name for record in verified.routed_net_records
    ]
    assert artifacts.route_search_summary.route_count == 3
    # Slice 2c: the artifacts carry the plan as the plain mapping its consumers
    # (photonic_router.flow_verification, the crossing report) still read.
    assert isinstance(artifacts.crossing_plan_info, dict)
    assert artifacts.crossing_plan_info == pipeline.state.crossing_plan_info.to_dict()
    # Realization happened on the session's layout, not on the unrouted one.
    assert pipeline.state.routed_layout.get_polygons()


def test_phase5_stores_the_static_cells_and_the_run_plan(pipeline: Pipeline) -> None:
    final = pipeline.through(5)
    state = pipeline.state

    assert state.run.route_order == [job.net_id for job in final.route_jobs]
    assert state.search.bookkeeping.route_order is state.run.route_order
    # the blockers before port reservations: the raw static cells plus the runways
    assert (
        state.static_cells.blocked_cells_before_port_reservations
        >= state.kernel.static.raw_static_cells | state.ports.normal_runway_cells
    )
    with pytest.raises(FrozenInstanceError):
        state.run.route_order = []  # type: ignore[misc]


def test_phase4_stores_the_port_footprints_as_one_frozen_record(pipeline: Pipeline) -> None:
    pipeline.through(4)
    ports = pipeline.state.ports

    assert set(ports.access_cells_by_spec) == set(pipeline.state.endpoints.ports_by_spec)
    assert ports.normal_runway_cells == set().union(*ports.runway_cells_by_spec.values())
    assert ports.foreign_keepout_cells_by_spec == {}  # the keep-out is off by default
    with pytest.raises(FrozenInstanceError):
        ports.state_lane_offsets = {}  # type: ignore[misc]


def test_a_dense_port_window_reaches_halfway_to_each_neighbour() -> None:
    window = crossing_plan_stage_module._window_between_neighbours

    assert window([0.0, 4.0, 10.0], 1) == (2.0, 7.0)
    # at the ends the open side is as wide as the side of the one neighbour
    assert window([0.0, 4.0, 10.0], 0) == (-2.0, 2.0)
    assert window([0.0, 4.0, 10.0], 2) == (7.0, 13.0)


def test_the_kernel_setup_indexes_its_static_rectangles_by_row(pipeline: Pipeline) -> None:
    kernel = replace(
        pipeline.through(2),
        static=RawStaticGeometry(
            raw_static_cells={(3, 1)},
            raw_static_rects_for_openings=[(0, 0, 2, 1), (2, 1, 5, 1), (-4, 2, -1, 2)],
            heater_opening_rects_for_openings=[],
        ),
    )

    assert kernel.raw_static_cells_by_y == {1: {3}}
    # overlapping ranges of one row merge; a rectangle outside the grid is dropped
    assert kernel.raw_static_rect_ranges_by_y == {0: [(0, 2)], 1: [(0, 5)]}
    assert kernel.heater_opening_rect_ranges_by_y == {}


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
