"""Stage 12: write the electrical routing debug SVG artifacts.

The two SVG exporters live in their own modules (``debug_grid_svg`` for the
grid-coordinate obstacle/route view, ``metal_snapshot_svg`` for the realized
physical-coordinate metal); this module is the thin stage function that
decides which artifacts to write and returns their paths, the same contract
``route_electrical_heaters`` used to inline.
"""

from __future__ import annotations

from pathlib import Path

from .debug_grid_svg import export_electrical_debug_svg
from .metal_snapshot_svg import export_electrical_metal_snapshot_svg
from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    IndividualEscapeTopologyResult,
    PadPlan,
    TerminalPairGroup,
)


def write_debug_artifacts(
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
) -> dict[str, str]:
    """Write the debug SVGs for a routed run and return their paths by key.

    Returns an empty dict when ``debug_dir`` is None; otherwise always writes
    the grid-coordinate common-bus SVG, and additionally the physical-
    coordinate metal snapshot SVG when ``routed_component`` is not None.
    """

    artifacts: dict[str, str] = {}
    if debug_dir is None:
        return artifacts

    debug_path = Path(debug_dir) / "electrical" / f"{debug_prefix}_common_bus.svg"
    export_electrical_debug_svg(
        debug_path,
        obstacle_map,
        terminal_groups,
        common_bus,
        common_bus_escape=common_bus_escape,
        individual_topology=individual_topology,
        detailed_bundle_routes=detailed_bundle_routes,
        pad_plan=pad_plan,
    )
    artifacts["common_bus_svg"] = str(debug_path)

    if routed_component is not None:
        metal_snapshot_path = Path(debug_dir) / "electrical" / f"{debug_prefix}_metal_snapshot.svg"
        export_electrical_metal_snapshot_svg(
            metal_snapshot_path,
            routed_component,
            obstacle_map,
            terminal_groups,
            pad_plan,
            config,
        )
        artifacts["metal_snapshot_svg"] = str(metal_snapshot_path)

    return artifacts
