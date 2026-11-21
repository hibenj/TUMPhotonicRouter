"""The routing session shell: construction, `prepare`, the nine-phase `run`
and the module-level `route_nets_rust` entry point.

The phase bodies and their helpers live in the sibling stage modules as
module-level functions. The session validates the options
(`options.SessionOptions`) into `self.settings` (frozen
`settings.SessionSettings`), runs phases 1 and 2, builds the shared record
(`state.SessionState`) from their products and calls the remaining phases
of `stages.py`, whose Protocols declare each phase's inputs and its product.
"""

from __future__ import annotations

import importlib

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.records.route_rust_types import RustRouteDebugArtifacts
from photonic_router.routing import (
    crossing_plan_stage,
    detour_pass,
    dispatch,
    finalize,
    handoff,
    obstacle_context,
    realize,
    router_setup,
    stage_observer,
    verify_repair,
)
from photonic_router.routing import (
    route_jobs as route_jobs_stage,  # `route_jobs` is a local name in `run`
)
from photonic_router.routing.options import SessionOptions
from photonic_router.routing.settings import (
    SessionSettings,
)
from photonic_router.routing.state import SessionState

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend


class RouteNetsRustSession:
    """One routing run: the settings, the state the phases share, and `run`."""

    def __init__(
        self,
        unrouted_layout: Component,
        schematic: Schematic,
        options: SessionOptions | None = None,
    ) -> None:
        """Validate the options and set up an empty session for the design.

        Args:
            unrouted_layout: the component with placed instances and no routes.
            schematic: the schematic whose nets are routed.
            options: the caller's choices (`options.py`); None is the default
                session.
        """
        self.settings = SessionSettings.from_options(
            unrouted_layout, schematic, options if options is not None else SessionOptions()
        )

        rust_backend = _load_rust_backend()
        if rust_backend is None:
            raise RuntimeError(
                "Rust router backend is not available. Build it with `cargo build` "
                "or `maturin develop` so photonic_router._rust can be imported."
            )

        self._rust_backend = rust_backend

    def prepare(self) -> SessionState:
        """Run phases 1 and 2 and return the state the later phases share.

        Everything a session sets up before its first search that does not
        depend on the nets: the static obstacle map, the grid and the
        kernel's router.
        """
        settings = self.settings
        timings_s: dict[str, float] = {}
        context = obstacle_context.build_static_obstacle_context(
            settings, self._rust_backend, timings_s
        )
        stage_observer.notify("obstacle_map", settings, context)
        kernel = router_setup.configure_router_and_grid(
            settings, self._rust_backend, context, timings_s
        )
        routed_layout = settings.unrouted_layout.copy()
        routed_layout.name = "routed_layout_rust"
        state = SessionState.after_setup(
            routed_layout=routed_layout,
            rust_backend=self._rust_backend,
            timings_s=timings_s,
            obstacles=context,
            kernel=kernel,
        )
        stage_observer.notify("router_setup", settings, state)
        return state

    def run(self) -> tuple[Component, RustRouteDebugArtifacts]:
        """The routing phases of `stages.py`, in order (nine, plus the optional detour pass).

        Phases 1 and 2 return the products the state is built from
        (`prepare`); each later phase takes the run's frozen inputs and its
        state, and the value a phase returns is the input of a later one
        (`stages.py` names the types).
        """
        settings, state = self.settings, self.prepare()
        jobs = route_jobs_stage.build_route_jobs_and_fanout_clustering(
            settings, state, settings.schematic.netlist.routes
        )
        route_jobs_stage.apply_long_straight_fanout_exemption(settings, state)
        planned = crossing_plan_stage.build_crossing_plan_and_port_footprints(
            settings,
            state,
            jobs.route_jobs,
            jobs.endpoint_port_specs_by_instance,
            jobs.dense_port_runway_length_by_spec,
        )
        final = handoff.finalize_route_jobs_and_static_handoff(
            settings, state, planned.route_jobs, planned.foreign_port_keepout_cells_by_instance
        )
        stage_observer.notify("handoff", settings, state)
        dispatch.dispatch_native_routing(settings, state, final.route_jobs)
        routed = finalize.finalize_routing_results(
            settings, state, final.route_jobs, final.astar_start_s
        )
        if settings.detour_pass_factor > 0:
            routed = detour_pass.run_detour_pass(settings, state, routed)
        verified = verify_repair.repair_and_verify_final_geometry(
            settings, state, routed.routed_net_records
        )
        debug_artifacts = realize.realize_and_assemble_debug_artifacts(
            settings, state, final.route_jobs, routed, verified
        )
        return state.routed_layout, debug_artifacts


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    options: SessionOptions | None = None,
) -> tuple[Component, RustRouteDebugArtifacts]:
    """Route the nets of a schematic and return the layout and the debug record.

    The session builds the static obstacle map of the layout, routes every
    net with the Rust kernel (negotiated rip-up and reroute), verifies and
    repairs the result and, unless `options.realization.defer_realization`
    is set, adds one polygon per routed net to a copy of the layout.
    """
    session = RouteNetsRustSession(unrouted_layout, schematic, options)
    return session.run()
