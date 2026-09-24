"""`SessionSettings`/`SessionState`/pipeline builders shared across the tests.

ExecPlan Milestone 6, Slice 2. `settings_for_test` moved here unchanged from
`tests/test_route_rust_opened_cells.py`; `state_for_test` is its `SessionState`
counterpart, for the same "session assembled with `object.__new__`" idiom;
`Pipeline`/`pipeline_for_test` is `tests/test_routing_stages.py`'s `_Pipeline`,
moved unchanged, running the nine `translation.routing` stages over the
three-net synthetic scenario in `tests.fixtures.synthetic_layouts`.
"""

from __future__ import annotations

from typing import Any
from types import SimpleNamespace
from uuid import uuid4

import pytest
from gdsfactory.component import Component

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig, _load_rust_backend
from translation.routing import crossing_plan_stage as crossing_plan_stage_module
from translation.routing import dispatch as dispatch_module
from translation.routing import finalize as finalize_module
from translation.routing import handoff as handoff_module
from translation.routing import obstacle_context as obstacle_context_module
from translation.routing import realize as realize_module
from translation.routing import route_jobs as route_jobs_module
from translation.routing import router_setup as router_setup_module
from translation.routing import verify_repair as verify_repair_module
from translation.routing.settings import SessionSettings
from translation.routing.state import SessionState

from .synthetic_layouts import (
    GRID_HEIGHT,
    GRID_WIDTH,
    ObstacleMapStandIn,
    port_from_instance,
    three_net_schematic,
)


def settings_for_test(**overrides: Any) -> SessionSettings:
    """Build a `SessionSettings` for a session assembled with `object.__new__`.

    Milestone 5 Slice 2a moved the session's keyword-derived attributes into one
    frozen `SessionSettings`, so a test that used to write `session.<attr> = ...`
    writes `session.settings = settings_for_test(<attr>=...)` instead. The
    overrides are the constructor's own keywords, so they go through the same
    validation and normalisation the production session uses.
    """
    return SessionSettings.from_arguments(
        SimpleNamespace(),  # unrouted_layout: untouched by the unit tests below
        SimpleNamespace(),  # schematic: untouched by the unit tests below
        **overrides,
    )


def state_for_test(**overrides: Any) -> SessionState:
    """Build a `SessionState` directly from its own keyword overrides.

    `SessionState`'s fields already default sensibly (unlike `SessionSettings`,
    which normalises its arguments through `from_arguments`), so this is a thin
    alias -- it exists so a test that hands a fake session both its settings
    and its state can write `settings_for_test(...)`/`state_for_test(...)` as
    one paired convention.
    """
    return SessionState(**overrides)


class Pipeline:
    """The nine phases over one synthetic scenario, each run at most once."""

    def __init__(self, monkeypatch: pytest.MonkeyPatch, **overrides: Any) -> None:
        monkeypatch.setattr(
            obstacle_context_module,
            "build_static_obstacle_map",
            lambda _component, config=None: ObstacleMapStandIn(),
        )
        monkeypatch.setattr(route_jobs_module, "get_port_from_instance", port_from_instance)
        layout = Component(f"routing_stage_layout_{uuid4().hex}")
        self.settings = SessionSettings.from_arguments(
            layout,
            three_net_schematic(),
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
            # `pop`ped rather than fixed, so a test about the A* iteration
            # budget itself can raise it above the Weighted-A* cap; every
            # other caller still gets 20,000.
            max_iterations=overrides.pop("max_iterations", 20_000),
            # Turns the pipeline timers on, so the phases that carry a clock
            # (5's search start, 7's elapsed search time) carry a real one.
            collect_route_stats=True,
            **overrides,
        )
        rust_backend = _load_rust_backend()
        if rust_backend is None:  # pragma: no cover - the kernel is built in CI
            pytest.skip("Rust router backend unavailable.")
        self.state = state_for_test(rust_backend=rust_backend, routed_layout=layout.copy())
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


def pipeline_for_test(monkeypatch: pytest.MonkeyPatch, **overrides: Any) -> Pipeline:
    return Pipeline(monkeypatch, **overrides)
