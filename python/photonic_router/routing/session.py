"""The routing session shell: construction, the nine-phase `run` and the
module-level `route_nets_rust` entry point.

Milestone 5 Slice 1 moved every phase body and its stage-private helpers into
the sibling stage modules of this package, as module-level functions taking the
session as their first argument, with a one-line forwarder per moved method on
this class. Slice 2b removed those 123 forwarders: a stage calls a helper of its
own module by name and a helper of another stage through an import from that
stage's module, so the only entry points left on the class are `__init__` and
`run`. The pipeline timers moved to `timing.py` for the same reason.

Slice 2b also split what the session carried into the two records a stage now
takes: `self.settings` (frozen `settings.SessionSettings`, Slice 2a) is the run's
inputs, `self.state` (mutable `state.SessionState`) is everything the run
computes -- one documented field per per-run attribute, with the writing and
reading phases named. The class itself owns nothing else: it validates the
options (`options.SessionOptions`), builds the two records and calls the nine phases of `stages.py`, whose
Protocols declare each phase's inputs and its product."""

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


class _RouteNetsRustSession:
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

        routed_layout = self.settings.unrouted_layout.copy()
        routed_layout.name = "routed_layout_rust"

        # Slice 2b: everything the run computes lives in one typed record, so a
        # stage takes `(settings, state)` instead of the session itself. The
        # remaining two fields are what the constructor produces.
        self.state = SessionState(rust_backend=rust_backend, routed_layout=routed_layout)

    def run(self) -> tuple[Component, RustRouteDebugArtifacts]:
        """The routing phases of `stages.py`, in order (nine, plus the optional detour pass).

        Each takes the run's frozen inputs and its mutable state; the value a
        phase returns is the input of a later one (`stages.py` names the types).
        """
        settings, state = self.settings, self.state
        context = obstacle_context.build_static_obstacle_context(settings, state)
        router_setup.configure_router_and_grid(settings, state, context.obstacle_map)
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
            context.obstacle_map,
            context.crossing_device_info,
        )
        final = handoff.finalize_route_jobs_and_static_handoff(
            settings,
            state,
            planned.route_jobs,
            planned.foreign_port_keepout_cells_by_instance,
            context.obstacle_map,
        )
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
            settings,
            state,
            final.route_jobs,
            verified.routed_net_records,
            verified.illegal_realized_crossings,
            context.obstacle_map,
            context.obstacle_svg,
            routed.astar_elapsed_s,
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
    session = _RouteNetsRustSession(unrouted_layout, schematic, options)
    return session.run()
