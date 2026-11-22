"""`SessionSettings`/`SessionState`/pipeline builders shared across the tests.

ExecPlan Milestone 6, Slice 2. `settings_for_test` moved here unchanged from
`tests/test_route_rust_opened_cells.py`; `state_for_test` is its `SessionState`
counterpart, for the same "session assembled with `object.__new__`" idiom;
`Pipeline`/`pipeline_for_test` is `tests/test_routing_stages.py`'s `_Pipeline`,
moved unchanged, running the nine `photonic_router.routing` stages over the
three-net synthetic scenario in `tests.fixtures.synthetic_layouts`.
"""

from __future__ import annotations

from dataclasses import fields, replace
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

import pytest
from gdsfactory.component import Component
from photonic_router.electrical import (
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
    route_electrical_heaters,
)
from photonic_router.flow import load_benchmark
from photonic_router.records.layout_from_schematic import layout_from_schematic
from photonic_router.routing import crossing_plan_stage as crossing_plan_stage_module
from photonic_router.routing import dispatch as dispatch_module
from photonic_router.routing import finalize as finalize_module
from photonic_router.routing import handoff as handoff_module
from photonic_router.routing import obstacle_context as obstacle_context_module
from photonic_router.routing import realize as realize_module
from photonic_router.routing import route_jobs as route_jobs_module
from photonic_router.routing import router_setup as router_setup_module
from photonic_router.routing import verify_repair as verify_repair_module
from photonic_router.routing.options import SessionOptions
from photonic_router.routing.products import (
    ClearanceRadii,
    DenseRunways,
    EndpointIndex,
    FanoutStubs,
    GridFrame,
    KernelSetup,
    ObstacleContext,
    PortFootprints,
    PortLanes,
    RawStaticGeometry,
    RunPlan,
    StaticHandoffCells,
)
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.state import SearchLog, SessionState
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig, load_rust_backend

from tests.fixtures.synthetic_layouts import (
    GRID_HEIGHT,
    GRID_WIDTH,
    ObstacleMapStandIn,
    port_from_instance,
    three_net_schematic,
)

_OPTION_GROUPS = ("search", "crossings", "port_access", "debug", "realization")


def session_options(**values: Any) -> SessionOptions:
    """Build `SessionOptions` from flat field names.

    A test names the few options it cares about; this puts each one into the
    group of `photonic_router.routing.options` that declares it, e.g.
    `session_options(max_iterations=5, debug_dir=path)`. An unknown name
    raises, so a renamed option cannot be passed silently.
    """
    top_level = {f.name for f in fields(SessionOptions)} - set(_OPTION_GROUPS)
    grouped: dict[str, dict[str, Any]] = {}
    top: dict[str, Any] = {}
    for name, value in values.items():
        if name in top_level:
            top[name] = value
            continue
        owners = [
            group
            for group in _OPTION_GROUPS
            if name in {f.name for f in fields(type(getattr(SessionOptions(), group)))}
        ]
        if len(owners) != 1:
            raise TypeError(f"session_options: {name!r} is not an option of a routing session")
        grouped.setdefault(owners[0], {})[name] = value
    defaults = SessionOptions()
    return replace(
        defaults,
        **top,
        **{group: replace(getattr(defaults, group), **chosen) for group, chosen in grouped.items()},
    )


def settings_for_test(**overrides: Any) -> SessionSettings:
    """Build a `SessionSettings` for a session assembled with `object.__new__`.

    A test that needs a fake session writes
    `session.settings = settings_for_test(<option>=...)`. The overrides are
    the flat option names of `session_options`, so they go through the same
    validation and normalisation the production session uses.
    """
    return SessionSettings.from_options(
        SimpleNamespace(),  # unrouted_layout: untouched by the unit tests below
        SimpleNamespace(),  # schematic: untouched by the unit tests below
        session_options(**overrides),
    )


_PRODUCT_GROUPS: dict[str, type] = {
    "frame": GridFrame,
    "radii": ClearanceRadii,
    "lanes": PortLanes,
    "static": RawStaticGeometry,
}


#: the slots of the records of phases 3 to 5 (`products.py`)
_STEP_RECORDS: dict[str, Any] = {
    "endpoints": EndpointIndex,
    "fanout": FanoutStubs,
    "runways": DenseRunways,
    "ports": PortFootprints,
    "static_cells": StaticHandoffCells,
    "run": RunPlan,
}


def _placeholders() -> dict[str, Any]:
    """Fresh placeholders for a state that never ran phases 1 and 2.

    A field that is not listed is a cell count and defaults to 0.
    """
    return {
        "obstacle_map": None,
        "crossing_device_info": {},
        "obstacle_svg": None,
        "grid": None,
        "resolved_obstacle_config": None,
        "debug_path": None,
        "router": None,
        "primitive_cfg": None,
        "astar_cfg": None,
        "clearance_policy": None,
        "route_clearance_um": 0.0,
        "origin_x_um": 0.0,
        "origin_y_um": 0.0,
        "raw_static_cells": set(),
        "raw_static_rects_for_openings": [],
        "heater_opening_rects_for_openings": [],
    }


def _product(record: type, values: dict[str, Any], **nested: Any) -> Any:
    """Build one product record, taking its fields out of `values`."""
    placeholders = _placeholders()
    own: dict[str, Any] = dict(nested)
    for item in fields(record):
        if item.name not in own:
            own[item.name] = values.pop(item.name, placeholders.get(item.name, 0))
    return record(**own)


def state_for_test(**overrides: Any) -> SessionState:
    """Build a `SessionState` from flat field names, without running a phase.

    A test names the few values it cares about. A name that belongs to the
    product of phase 1 or 2 (`grid`, `router`, `bend_radius_cells`, ...) goes
    into `state.obstacles` or `state.kernel`, a name of one of phase 3's step
    records (`anchor_net_ids`, `ports_by_spec`, ...) or of the search's log
    (`bookkeeping`, ...) into that record; every other name is a field of the
    state itself. What is not named is an inert
    placeholder.
    """
    values = dict(overrides)
    obstacles = _product(ObstacleContext, values)
    groups = {name: _product(record, values) for name, record in _PRODUCT_GROUPS.items()}
    kernel = _product(KernelSetup, values, **groups)
    for slot, record in _STEP_RECORDS.items():
        named = {item.name: values.pop(item.name) for item in fields(record) if item.name in values}
        if named:
            values[slot] = replace(record.empty(), **named)
    logged = {item.name: values.pop(item.name) for item in fields(SearchLog) if item.name in values}
    if logged:
        values["search"] = SearchLog(**logged)
    values.setdefault("routed_layout", None)
    values.setdefault("rust_backend", None)
    return SessionState(obstacles=obstacles, kernel=kernel, **values)


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
        self.settings = SessionSettings.from_options(
            layout,
            three_net_schematic(),
            session_options(
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
            ),
        )
        rust_backend = load_rust_backend()
        if rust_backend is None:  # pragma: no cover - the kernel is built in CI
            pytest.skip("Rust router backend unavailable.")
        self._rust_backend = rust_backend
        self._routed_layout = layout.copy()
        self._timings_s: dict[str, float] = {}
        self._state: SessionState | None = None
        self._products: dict[int, Any] = {}

    @property
    def state(self) -> SessionState:
        """The shared state, built from the products of phases 1 and 2."""
        if self._state is None:
            self._state = SessionState.after_setup(
                routed_layout=self._routed_layout,
                rust_backend=self._rust_backend,
                timings_s=self._timings_s,
                obstacles=self.through(1),
                kernel=self.through(2),
            )
        return self._state

    def through(self, phase: int) -> Any:
        """The product of `phase`, running the phases before it once each."""

        for index in range(1, phase + 1):
            if index not in self._products:
                self._products[index] = self._run(index)
        return self._products[phase]

    def _run(self, phase: int) -> Any:
        settings = self.settings
        if phase == 1:
            return obstacle_context_module.build_static_obstacle_context(
                settings, self._rust_backend, self._timings_s
            )
        if phase == 2:
            return router_setup_module.configure_router_and_grid(
                settings, self._rust_backend, self.through(1), self._timings_s
            )
        state = self.state
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
            )
        if phase == 5:
            planned = self.through(4)
            return handoff_module.finalize_route_jobs_and_static_handoff(
                settings,
                state,
                planned.route_jobs,
                planned.foreign_port_keepout_cells_by_instance,
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
                settings, state, self.through(5).route_jobs, self.through(7), verified
            )
        raise AssertionError(f"no phase {phase}")


def pipeline_for_test(monkeypatch: pytest.MonkeyPatch, **overrides: Any) -> Pipeline:
    return Pipeline(monkeypatch, **overrides)


def route_electrical_benchmark_result(benchmark_name: str) -> ElectricalRoutingResult:
    """Route ``benchmark_name`` through the full electrical pipeline with a
    default `ElectricalRoutingConfig`.

    Backs the `electrical_benchmark_result` fixture in `tests/conftest.py` and
    the flow-level electrical tests in `tests/e2e/test_routing_flow_stats.py`.
    """

    schematic = load_benchmark(benchmark_name)
    component = layout_from_schematic(schematic)
    return route_electrical_heaters(component, schematic, ElectricalRoutingConfig())
