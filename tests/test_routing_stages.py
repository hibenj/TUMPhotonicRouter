"""One test per routing stage Protocol (`translation/routing/stages.py`).

Milestone 5 Slice 2c. Slice 2b declared the nine phases of `session.run()` as
`typing.Protocol`s; these tests exercise each phase on its own, on the smallest
synthetic scenario the existing session tests use (an empty layout, a dummy
netlist of three nets, a 30x20 one-micron grid from a stand-in obstacle builder),
with no benchmark and no schematic loading. Every phase gets the product of the
phases before it, so each test asserts on one phase's own product -- the frozen
dataclass it returns, or, for the two phases that return `None`, the state fields
it writes.

The last test is the conformance one: each phase function's signature is checked
against the `__call__` of its Protocol at run time, and the nine module-level
annotated assignments next to it are the same check for a type checker.
"""

from __future__ import annotations

import inspect
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from gdsfactory.component import Component
from gdsfactory.gpdk import get_generic_pdk
from photonic_router.static_obstacle_builder import (
    GridSpec,
    StaticObstacleMapConfig,
    _load_rust_backend,
)

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
from translation.routing.settings import SessionSettings
from translation.routing.state import SessionState

get_generic_pdk().activate()

GRID_WIDTH = 30
GRID_HEIGHT = 20

# port spec -> (centre in micrometres, orientation in degrees).
_PORTS: dict[str, tuple[tuple[float, float], float]] = {
    "left,o1": ((2.5, 4.5), 0.0),
    "left,o2": ((2.5, 9.5), 0.0),
    "left,o3": ((2.5, 14.5), 0.0),
    "right0,o1": ((27.5, 4.5), 180.0),
    "right1,o1": ((27.5, 9.5), 180.0),
    "right2,o1": ((27.5, 14.5), 180.0),
}
_LINKS = {
    "net_0": ("left,o1", "right0,o1"),
    "net_1": ("left,o2", "right1,o1"),
    "net_2": ("left,o3", "right2,o1"),
}


class _ObstacleMapStandIn:
    """What phase 1 expects of `build_static_obstacle_map`: a grid and no obstacle."""

    def __init__(self) -> None:
        self.grid = GridSpec(
            width=GRID_WIDTH,
            height=GRID_HEIGHT,
            grid_size_um=1.0,
            origin=(0.0, 0.0),
            die_bbox=(0.0, 0.0, float(GRID_WIDTH), float(GRID_HEIGHT)),
        )
        self.blocked_cells: set[tuple[int, int]] = set()
        self.raw_blocked_cells: set[tuple[int, int]] = set()
        self.port_open_cells: set[tuple[int, int]] = set()

    def export_debug_svg(self, path: Any) -> None:  # pragma: no cover - debug only
        path.write_text("<svg/>", encoding="utf-8")


def _schematic() -> Any:
    return SimpleNamespace(
        netlist=SimpleNamespace(
            routes={
                name: SimpleNamespace(links={source: target})
                for name, (source, target) in _LINKS.items()
            },
            instances={
                "left": SimpleNamespace(component="left_mmi"),
                "right0": SimpleNamespace(component="right_gc"),
                "right1": SimpleNamespace(component="right_gc"),
                "right2": SimpleNamespace(component="right_gc"),
            },
        )
    )


def _port_from_instance(_layout: Any, instance: str, port: str) -> SimpleNamespace:
    center, orientation = _PORTS[f"{instance},{port}"]
    return SimpleNamespace(center=center, orientation=orientation)


class _Pipeline:
    """The nine phases over one synthetic scenario, each run at most once."""

    def __init__(self, monkeypatch: pytest.MonkeyPatch, **overrides: Any) -> None:
        monkeypatch.setattr(
            obstacle_context_module,
            "build_static_obstacle_map",
            lambda _component, config=None: _ObstacleMapStandIn(),
        )
        monkeypatch.setattr(route_jobs_module, "get_port_from_instance", _port_from_instance)
        layout = Component(f"routing_stage_layout_{uuid4().hex}")
        self.settings = SessionSettings.from_arguments(
            layout,
            _schematic(),
            obstacle_config=StaticObstacleMapConfig(
                obstacle_mode="rasterized_polygons",
                grid_size_um=1.0,
                security_margin_um=0.0,
                clearance_um=0.0,
                port_open_radius_um=0.0,
                die_bbox=(0.0, 0.0, float(GRID_WIDTH), float(GRID_HEIGHT)),
            ),
            route_width_um=0.5,
            allow_45_degree_turns=True,
            bend_radius_um=1.0,
            max_iterations=20_000,
            # Turns the pipeline timers on, so the phases that carry a clock
            # (5's search start, 7's elapsed search time) carry a real one.
            collect_route_stats=True,
            **overrides,
        )
        rust_backend = _load_rust_backend()
        if rust_backend is None:  # pragma: no cover - the kernel is built in CI
            pytest.skip("Rust router backend unavailable.")
        self.state = SessionState(rust_backend=rust_backend, routed_layout=layout.copy())
        self._products: dict[int, Any] = {}

    def through(self, phase: int) -> Any:
        """The product of `phase`, running the phases before it once each."""

        for index in range(1, phase + 1):
            if index not in self._products:
                self._products[index] = self._run(index)
        return self._products[phase]

    def _run(self, phase: int) -> Any:
        settings, state = self.settings, self.state
        if phase == 1:
            return obstacle_context_module.build_static_obstacle_context(settings, state)
        if phase == 2:
            return router_setup_module.configure_router_and_grid(
                settings, state, self.through(1).obstacle_map
            )
        if phase == 3:
            jobs = route_jobs_module.build_route_jobs_and_fanout_clustering(
                settings, state, settings.schematic.netlist.routes
            )
            route_jobs_module.apply_long_straight_fanout_exemption(settings, state)
            return jobs
        if phase == 4:
            jobs = self.through(3)
            return crossing_plan_stage_module.build_crossing_plan_and_port_footprints(
                settings,
                state,
                jobs.route_jobs,
                jobs.endpoint_port_specs_by_instance,
                jobs.dense_port_runway_length_by_spec,
                self.through(1).obstacle_map,
                self.through(1).crossing_device_info,
            )
        if phase == 5:
            planned = self.through(4)
            return handoff_module.finalize_route_jobs_and_static_handoff(
                settings,
                state,
                planned.route_jobs,
                planned.foreign_port_keepout_cells_by_instance,
                self.through(1).obstacle_map,
            )
        if phase == 6:
            return dispatch_module.dispatch_native_routing(
                settings, state, self.through(5).route_jobs
            )
        if phase == 7:
            final = self.through(5)
            self.through(6)
            return finalize_module.finalize_routing_results(
                settings, state, final.route_jobs, final.astar_start_s
            )
        if phase == 8:
            return verify_repair_module.repair_and_verify_final_geometry(
                settings, state, self.through(7).routed_net_records
            )
        if phase == 9:
            verified = self.through(8)
            return realize_module.realize_and_assemble_debug_artifacts(
                settings,
                state,
                self.through(5).route_jobs,
                verified.routed_net_records,
                verified.illegal_realized_crossings,
                self.through(1).obstacle_map,
                self.through(1).obstacle_svg,
                self.through(7).astar_elapsed_s,
            )
        raise AssertionError(f"no phase {phase}")


@pytest.fixture
def pipeline(monkeypatch: pytest.MonkeyPatch) -> _Pipeline:
    return _Pipeline(monkeypatch)


def test_phase1_obstacle_context_builder_returns_the_map_and_the_grid(pipeline: _Pipeline) -> None:
    context = pipeline.through(1)

    assert isinstance(context, stages.ObstacleContext)
    assert isinstance(context.obstacle_map, _ObstacleMapStandIn)
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


def test_phase2_router_setup_builds_the_kernel_router_into_the_state(pipeline: _Pipeline) -> None:
    assert pipeline.through(2) is None  # phase 2's product is the state

    state = pipeline.state
    assert state.router is not None
    assert hasattr(state.router, "route_many_with_repair_and_commit")
    assert state.primitive_cfg is not None
    assert state.astar_cfg is not None
    assert (state.grid_width, state.grid_height) == (GRID_WIDTH, GRID_HEIGHT)
    assert (state.origin_x_um, state.origin_y_um) == (0.0, 0.0)
    assert state.bend_radius_cells == 1  # bend_radius_um=1.0 over a 1 um grid
    assert state.raw_static_cells == set()  # the stand-in map blocks nothing
    # The realization grid spec is phase 5's, not this phase's (see state.py).
    assert state.realization_grid_spec is None


def test_phase3_route_job_builder_turns_the_netlist_into_jobs(pipeline: _Pipeline) -> None:
    result = pipeline.through(3)

    assert isinstance(result, stages.RouteJobsResult)
    assert [job.net_name for job in result.route_jobs] == list(_LINKS)
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
    pipeline: _Pipeline,
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
    pipeline: _Pipeline,
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
    pipeline: _Pipeline,
) -> None:
    assert pipeline.through(6) is None  # phase 6's product is the state

    state = pipeline.state
    assert sorted(state.route_bookkeeping.records_by_id) == [1, 2, 3]
    assert state.deferred_count == 0
    assert state.total_expanded_states >= 0
    assert len(state.route_bookkeeping.ordered_records()) == 3


def test_phase7_result_finalizer_returns_one_record_per_net(pipeline: _Pipeline) -> None:
    routed = pipeline.through(7)

    assert isinstance(routed, stages.RoutedRecords)
    assert [record.net_name for record in routed.routed_net_records] == list(_LINKS)
    assert routed.astar_elapsed_s > 0.0
    for record in routed.routed_net_records:
        assert record.total_length_um > 0.0
        assert len(record.corrected_centerline_um) >= 2


def test_phase8_geometry_verifier_finds_no_illegal_crossing_on_parallel_nets(
    pipeline: _Pipeline,
) -> None:
    routed = pipeline.through(7)
    verified = pipeline.through(8)

    assert isinstance(verified, stages.VerifiedRecords)
    assert verified.illegal_realized_crossings == []
    assert [record.net_name for record in verified.routed_net_records] == [
        record.net_name for record in routed.routed_net_records
    ]


def test_phase9_realizer_assembles_the_debug_artifacts_with_the_plan_as_a_mapping(
    pipeline: _Pipeline,
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
