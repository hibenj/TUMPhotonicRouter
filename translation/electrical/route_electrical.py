"""Orchestrate the electrical heater-routing pipeline as a stage sequence.

The literal stage sequence run by ``route_electrical_heaters``, after the
pattern of ``translation/routing/session.py::run`` -- one call per stage,
each returning the frozen product its ``stages.py`` Protocol names:

    1  TerminalExtractor      terminal_extraction.extract_heater_terminal_pairs
       -> tuple[TerminalPairGroup, ...]
    2  ObstacleMapBuilder     obstacle_extraction.build_electrical_obstacle_map
       -> ElectricalObstacleMap
    3  CommonBusRouter        common_bus_router.route_common_bus
       -> CommonBusRoutingResult
    4  CommonBusTrimmer       common_bus_router.trim_common_bus_to_connections
       -> tuple[ElectricalObstacleMap, CommonBusRoutingResult]
    5  EscapeTopologyPlanner  individual_topology.compute_individual_escape_topology
       -> IndividualEscapeTopologyResult
    6  PadPlanner             pad_slots.plan_pad_slots
       -> PadPlan
    7  PadSideReconciler      pad_side_reconciliation.reconcile_pad_sides
       -> PadSideReconciliation (re-runs stages 4-6 when it swaps a heater)
    8  BusEscapeRouter        escape_router.route_common_bus_escape
       -> CommonBusEscapeResult
    9  PadWireRouter          bundle_detail_router.route_detailed_bundles
       -> DetailedBundleRoutingResult
    10 MetalRealizer          metal_realization.realize_electrical_metal
       -> Component
    11 ElectricalVerifier     verification.verify_electrical_routing
       -> ElectricalVerificationResult
    12 DebugArtifactWriter    debug.write_debug_artifacts
       -> dict[str, str]

Stages 4 through 12 only run when their inputs' guard holds (no terminals, no
successful common bus, no pad plan, ...), the same guards the milestone-era
inline version used; each guard is noted at its call below.
"""

from __future__ import annotations

from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from .bundle_detail_router import route_detailed_bundles
from .common_bus_router import route_common_bus, trim_common_bus_to_connections
from .debug import write_debug_artifacts
from .escape_router import route_common_bus_escape
from .individual_topology import compute_individual_escape_topology
from .metal_realization import realize_electrical_metal
from .obstacle_extraction import build_electrical_obstacle_map
from .pad_side_reconciliation import reconcile_pad_sides
from .pad_slots import plan_pad_slots
from .terminal_extraction import extract_heater_terminal_pairs
from .types import (
    CommonBusRoutingResult,
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
)
from .verification import verify_electrical_routing


def route_electrical_heaters(
    component: Component,
    schematic: Schematic | None = None,
    config: ElectricalRoutingConfig | None = None,
    *,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "electrical",
) -> ElectricalRoutingResult:
    """Run the electrical heater-routing pipeline as its twelve named stages.

    Extracts heater terminal pairs, builds the electrical obstacle grid,
    routes one terminal per heater to the derived opposite-side common bus,
    plans abstract pad slots, reconciles a lone heater's pad-side mismatch,
    routes the remaining individual terminals to those slots, realizes metal
    polygons, verifies the result, and optionally writes debug SVGs.
    """

    config = config or ElectricalRoutingConfig()
    config.validate()

    # Stage 1: TerminalExtractor.
    terminal_groups = extract_heater_terminal_pairs(component, schematic, config)
    # Stage 2: ObstacleMapBuilder.
    obstacle_map = build_electrical_obstacle_map(component, terminal_groups, config)
    if not terminal_groups:
        routed_component = component.copy()
        return ElectricalRoutingResult(
            terminal_groups=terminal_groups,
            obstacle_map=obstacle_map,
            common_bus=CommonBusRoutingResult(
                bus_side=config.bus_side,
                bus=obstacle_map.bus,
                selected_terminals={},
                unselected_terminals={},
                routes=(),
                tree_cells=frozenset(),
                failed_heaters=(),
            ),
            routed_component=routed_component,
        )

    # Stage 3: CommonBusRouter.
    common_bus = route_common_bus(terminal_groups, obstacle_map, config)
    # Stage 4: CommonBusTrimmer.
    obstacle_map, common_bus = trim_common_bus_to_connections(
        obstacle_map,
        common_bus,
        config,
    )
    # Stage 5: EscapeTopologyPlanner (only once the bus itself succeeded).
    individual_topology = (
        compute_individual_escape_topology(obstacle_map, common_bus, config)
        if common_bus.success
        else None
    )
    # Stage 6: PadPlanner (needs a successful bus and a topology).
    pad_plan = (
        plan_pad_slots(common_bus, obstacle_map, config, individual_topology)
        if common_bus.success
        else None
    )
    # Stage 7: PadSideReconciler; re-runs stages 4-6 internally when a heater
    # is swapped, otherwise returns the four objects unchanged.
    reconciliation = reconcile_pad_sides(
        obstacle_map,
        common_bus,
        individual_topology,
        pad_plan,
        config,
    )
    obstacle_map = reconciliation.obstacle_map
    common_bus = reconciliation.common_bus
    individual_topology = reconciliation.individual_topology
    pad_plan = reconciliation.pad_plan

    # Stage 8: BusEscapeRouter (needs a pad plan).
    common_bus_escape = (
        route_common_bus_escape(obstacle_map, common_bus, pad_plan, config)
        if pad_plan is not None
        else None
    )
    # Stage 9: PadWireRouter (needs a successful escape, topology and pad plan).
    detailed_bundle_routes = (
        route_detailed_bundles(
            obstacle_map,
            common_bus,
            common_bus_escape,
            individual_topology,
            pad_plan,
            config,
        )
        if (
            pad_plan is not None
            and common_bus_escape is not None
            and individual_topology is not None
            and common_bus_escape.success
        )
        else None
    )
    # Stage 10: MetalRealizer (needs a successful bus).
    routed_component = (
        realize_electrical_metal(
            component,
            obstacle_map,
            common_bus,
            common_bus_escape,
            detailed_bundle_routes,
            pad_plan,
            config,
        )
        if common_bus.success
        else None
    )
    # Stage 11: ElectricalVerifier (needs realized metal).
    verification = (
        verify_electrical_routing(
            obstacle_map,
            common_bus,
            common_bus_escape,
            detailed_bundle_routes,
            pad_plan,
            config,
        )
        if routed_component is not None
        else None
    )
    # Stage 12: DebugArtifactWriter.
    artifacts = write_debug_artifacts(
        debug_dir,
        debug_prefix,
        obstacle_map=obstacle_map,
        terminal_groups=terminal_groups,
        common_bus=common_bus,
        common_bus_escape=common_bus_escape,
        individual_topology=individual_topology,
        detailed_bundle_routes=detailed_bundle_routes,
        pad_plan=pad_plan,
        routed_component=routed_component,
        config=config,
    )

    return ElectricalRoutingResult(
        terminal_groups=terminal_groups,
        obstacle_map=obstacle_map,
        common_bus=common_bus,
        pad_plan=pad_plan,
        common_bus_escape=common_bus_escape,
        individual_topology=individual_topology,
        detailed_bundle_routes=detailed_bundle_routes,
        routed_component=routed_component,
        verification=verification,
        debug_artifacts=artifacts,
    )
