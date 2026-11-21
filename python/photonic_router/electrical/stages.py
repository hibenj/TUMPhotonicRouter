"""The twelve electrical routing phases as declared interfaces.

Milestone 4 slice 4: one `typing.Protocol` per stage of
`route_electrical.route_electrical_heaters`, after the pattern of
`python/photonic_router/routing/stages.py`. Every Protocol's `__call__` matches its
implementing function's real signature exactly (parameter names, order,
defaults and keyword-only-ness); where the table below and the
implementation disagree, the implementation wins and the difference is noted
inline.

    1  TerminalExtractor      (component, schematic, config)                       -> tuple[TerminalPairGroup, ...]
    2  ObstacleMapBuilder     (component, terminal_groups, config)                 -> ElectricalObstacleMap
    3  CommonBusRouter        (terminal_groups, obstacle_map, config)              -> CommonBusRoutingResult
    4  CommonBusTrimmer       (obstacle_map, common_bus, config)                   -> tuple[ElectricalObstacleMap, CommonBusRoutingResult]
    5  EscapeTopologyPlanner  (obstacle_map, common_bus, config)                   -> IndividualEscapeTopologyResult
    6  PadPlanner             (common_bus, obstacle_map, config, escape_topology)  -> PadPlan
    7  PadSideReconciler      (obstacle_map, common_bus, individual_topology,
                                pad_plan, config)                                  -> PadSideReconciliation
    8  BusEscapeRouter        (obstacle_map, common_bus, pad_plan, config)         -> CommonBusEscapeResult
    9  PadWireRouter          (obstacle_map, common_bus, common_bus_escape,
                                topology, pad_plan, config)                        -> DetailedBundleRoutingResult
    10 MetalRealizer          (component, obstacle_map, common_bus,
                                common_bus_escape, detailed_bundle_routes,
                                pad_plan, config)                                  -> Component
    11 ElectricalVerifier     (obstacle_map, common_bus, common_bus_escape,
                                detailed_bundle_routes, pad_plan, config)          -> ElectricalVerificationResult
    12 DebugArtifactWriter    (debug_dir, debug_prefix, result, config)            -> dict[str, str]

Deviations from the brief's table, found by reading each implementation
before writing its Protocol:

- Stage 6 (``PadPlanner``, ``pad_slots.plan_pad_slots``): parameter order is
  ``(common_bus, obstacle_map, config, escape_topology)``, not
  ``(common_bus, obstacle_map, config, topology)``; the fourth parameter is
  named ``escape_topology`` and defaults to ``None``.
- Stage 9 (``PadWireRouter``, ``bundle_detail_router.route_detailed_bundles``):
  the escape parameter is named ``common_bus_escape``, not ``escape``, and
  the topology parameter is named ``topology``; both are plain positional
  parameters, not keyword-only, matching the table's order.
- Stage 10/11 (``MetalRealizer``, ``ElectricalVerifier``): the escape and
  bundle parameters are named ``common_bus_escape`` and
  ``detailed_bundle_routes``, not ``escape``/``bundles``; order matches the
  table.
- Stage 12 (``DebugArtifactWriter``): this stage function did not exist
  before this slice (Part 1.3 of the brief extracted it from the inline
  debug-export block of ``route_electrical_heaters``); its signature was
  designed for this slice rather than read from an existing implementation.
  It takes the ``ElectricalRoutingResult`` of stages 1 to 11 (still without
  debug artifacts) and returns the ``artifacts`` dict exactly as the inline
  block used to build it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.electrical.types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
    ElectricalVerificationResult,
    IndividualEscapeTopologyResult,
    PadPlan,
    PadSideReconciliation,
    TerminalPairGroup,
)


class TerminalExtractor(Protocol):
    """Stage 1: extract each heater's two interchangeable electrical terminals."""

    def __call__(
        self,
        component: Component,
        schematic: Schematic | None = None,
        config: ElectricalRoutingConfig | None = None,
    ) -> tuple[TerminalPairGroup, ...]:
        """Return one two-terminal group per heater instance of ``component``.

        Implemented by ``terminal_extraction.extract_heater_terminal_pairs``.
        """
        ...


class ObstacleMapBuilder(Protocol):
    """Stage 2: build the layer-filtered electrical obstacle grid."""

    def __call__(
        self,
        component: Component,
        terminal_groups: tuple[TerminalPairGroup, ...],
        config: ElectricalRoutingConfig,
    ) -> ElectricalObstacleMap:
        """Return the obstacle grid built from the configured metal and heater layers.

        Implemented by ``obstacle_extraction.build_electrical_obstacle_map``.
        """
        ...


class CommonBusRouter(Protocol):
    """Stage 3: connect exactly one terminal from each heater to the common bus."""

    def __call__(
        self,
        terminal_groups: tuple[TerminalPairGroup, ...],
        obstacle_map: ElectricalObstacleMap,
        config: ElectricalRoutingConfig,
    ) -> CommonBusRoutingResult:
        """Return the bus routes and the terminal chosen for the bus per heater.

        Implemented by ``common_bus_router.route_common_bus``. Heaters that
        could not be connected are listed in the result's ``failed_heaters``.
        """
        ...


class CommonBusTrimmer(Protocol):
    """Stage 4: limit the realized/debug bus stripe to the span actually used.

    ``keep_span_to_x_um`` (keyword-only, default ``None``) widens the
    trimmed span to reach exactly that x, so a second call after the pad
    plan exists can keep the stripe reaching the outer edge of the escape's
    band under the bus pad slot (``pad_slots.common_bus_stripe_reach_x_um``).
    """

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        config: ElectricalRoutingConfig,
        *,
        keep_span_to_x_um: float | None = None,
    ) -> tuple[ElectricalObstacleMap, CommonBusRoutingResult]:
        """Return the obstacle map and bus result with the stripe trimmed.

        Implemented by ``common_bus_router.trim_common_bus_to_connections``.
        """
        ...


class EscapeTopologyPlanner(Protocol):
    """Stage 5: infer coarse individual escape corridors before pad assignment."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        config: ElectricalRoutingConfig,
    ) -> IndividualEscapeTopologyResult:
        """Return coarse escape routes and bundles for the unselected terminals.

        Implemented by ``individual_topology.compute_individual_escape_topology``.
        """
        ...


class PadPlanner(Protocol):
    """Stage 6: assign used electrical nets to legal pad-pitch slots.

    ``escape_topology`` (not ``topology``) is the implementation's fourth
    parameter name; it defaults to None when no topology is available yet.
    """

    def __call__(
        self,
        common_bus: CommonBusRoutingResult,
        obstacle_map: ElectricalObstacleMap,
        config: ElectricalRoutingConfig,
        escape_topology: IndividualEscapeTopologyResult | None = None,
    ) -> PadPlan:
        """Return the abstract pad-slot plan; no pad geometry is created.

        Implemented by ``pad_slots.plan_pad_slots``.
        """
        ...


class PadSideReconciler(Protocol):
    """Stage 7: swap a lone heater's roles when its exit points away from its pad.

    ``keep_span_to_x_um`` (default ``None``) is threaded into the internal
    re-trim (``trim_common_bus_to_connections``) so a swap-triggered
    recompute keeps the stripe reaching the escape band's outer edge under
    the bus pad slot too.
    """

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        individual_topology: IndividualEscapeTopologyResult | None,
        pad_plan: PadPlan | None,
        config: ElectricalRoutingConfig,
        keep_span_to_x_um: float | None = None,
    ) -> PadSideReconciliation:
        """Return the pipeline objects after any role swap.

        Implemented by ``pad_side_reconciliation.reconcile_pad_sides``. The
        result's ``swapped_heater_ids`` names the heaters that were swapped.
        """
        ...


class BusEscapeRouter(Protocol):
    """Stage 8: route the common bus tree to its assigned abstract pad slot."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        pad_plan: PadPlan,
        config: ElectricalRoutingConfig,
    ) -> CommonBusEscapeResult:
        """Return the grid path from the bus tree to the bus's pad slot.

        Implemented by ``escape_router.route_common_bus_escape``. A failed
        escape is reported through the result's ``success`` and ``reason``.
        """
        ...


class PadWireRouter(Protocol):
    """Stage 9: route every individual pad wire as an L, or a Z fallback."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        common_bus_escape: CommonBusEscapeResult,
        topology: IndividualEscapeTopologyResult,
        pad_plan: PadPlan,
        config: ElectricalRoutingConfig,
    ) -> DetailedBundleRoutingResult:
        """Return the detailed centerline route of every individual pad wire.

        Implemented by ``bundle_detail_router.route_detailed_bundles``. Wires
        that could not be routed are in the result's ``failed_routes``.
        """
        ...


class MetalRealizer(Protocol):
    """Stage 10: realize every routed path as metal polygons on the layout."""

    def __call__(
        self,
        component: Component,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        common_bus_escape: CommonBusEscapeResult | None,
        detailed_bundle_routes: DetailedBundleRoutingResult | None,
        pad_plan: PadPlan | None,
        config: ElectricalRoutingConfig,
    ) -> Component:
        """Return a copy of ``component`` with the routed metal polygons added.

        Implemented by ``metal_realization.realize_electrical_metal``.
        """
        ...


class ElectricalVerifier(Protocol):
    """Stage 11: verify the realized electrical geometry's contracts."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        common_bus_escape: CommonBusEscapeResult | None,
        detailed_bundle_routes: DetailedBundleRoutingResult | None,
        pad_plan: PadPlan | None,
        config: ElectricalRoutingConfig,
    ) -> ElectricalVerificationResult:
        """Return the verification issues and quality metrics of the routed run.

        Implemented by ``verification.verify_electrical_routing``.
        """
        ...


class DebugArtifactWriter(Protocol):
    """Stage 12: write the debug SVGs for a routed run, keyed by artifact name."""

    def __call__(
        self,
        debug_dir: str | Path | None,
        debug_prefix: str,
        result: ElectricalRoutingResult,
        config: ElectricalRoutingConfig,
    ) -> dict[str, str]:
        """Return the written files' paths by artifact key.

        Implemented by ``debug.write_debug_artifacts``. Nothing is written and
        the dict is empty when ``debug_dir`` is None.
        """
        ...
