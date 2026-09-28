"""The twelve electrical routing phases as declared interfaces.

Milestone 4 slice 4: one `typing.Protocol` per stage of
`route_electrical.route_electrical_heaters`, after the pattern of
`translation/routing/stages.py`. Every Protocol's `__call__` matches its
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
    12 DebugArtifactWriter    (debug_dir, debug_prefix, *, obstacle_map,
                                terminal_groups, common_bus, common_bus_escape,
                                individual_topology, detailed_bundle_routes,
                                pad_plan, routed_component, config)                -> dict[str, str]

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
  It takes ``debug_dir`` and ``debug_prefix`` positionally and every other
  input keyword-only, and returns the ``artifacts`` dict exactly as the
  inline block used to build it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
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
    ) -> tuple[TerminalPairGroup, ...]: ...


class ObstacleMapBuilder(Protocol):
    """Stage 2: build the layer-filtered electrical obstacle grid."""

    def __call__(
        self,
        component: Component,
        terminal_groups: tuple[TerminalPairGroup, ...],
        config: ElectricalRoutingConfig,
    ) -> ElectricalObstacleMap: ...


class CommonBusRouter(Protocol):
    """Stage 3: connect exactly one terminal from each heater to the common bus."""

    def __call__(
        self,
        terminal_groups: tuple[TerminalPairGroup, ...],
        obstacle_map: ElectricalObstacleMap,
        config: ElectricalRoutingConfig,
    ) -> CommonBusRoutingResult: ...


class CommonBusTrimmer(Protocol):
    """Stage 4: limit the realized/debug bus stripe to the span actually used."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        config: ElectricalRoutingConfig,
    ) -> tuple[ElectricalObstacleMap, CommonBusRoutingResult]: ...


class EscapeTopologyPlanner(Protocol):
    """Stage 5: infer coarse individual escape corridors before pad assignment."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        config: ElectricalRoutingConfig,
    ) -> IndividualEscapeTopologyResult: ...


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
    ) -> PadPlan: ...


class PadSideReconciler(Protocol):
    """Stage 7: swap a lone heater's roles when its exit points away from its pad."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        individual_topology: IndividualEscapeTopologyResult | None,
        pad_plan: PadPlan | None,
        config: ElectricalRoutingConfig,
    ) -> PadSideReconciliation: ...


class BusEscapeRouter(Protocol):
    """Stage 8: route the common bus tree to its assigned abstract pad slot."""

    def __call__(
        self,
        obstacle_map: ElectricalObstacleMap,
        common_bus: CommonBusRoutingResult,
        pad_plan: PadPlan,
        config: ElectricalRoutingConfig,
    ) -> CommonBusEscapeResult: ...


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
    ) -> DetailedBundleRoutingResult: ...


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
    ) -> Component: ...


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
    ) -> ElectricalVerificationResult: ...


class DebugArtifactWriter(Protocol):
    """Stage 12: write the debug SVGs for a routed run, keyed by artifact name."""

    def __call__(
        self,
        debug_dir: str | Path | None,
        debug_prefix: str,
        *,
        obstacle_map: ElectricalObstacleMap,
        terminal_groups: tuple[TerminalPairGroup, ...],
        common_bus: CommonBusRoutingResult,
        common_bus_escape: CommonBusEscapeResult | None,
        individual_topology: IndividualEscapeTopologyResult | None,
        detailed_bundle_routes: DetailedBundleRoutingResult | None,
        pad_plan: PadPlan | None,
        routed_component: object | None,
        config: ElectricalRoutingConfig,
    ) -> dict[str, str]: ...
