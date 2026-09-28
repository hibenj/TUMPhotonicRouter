"""Helpers shared by the electrical routing debug SVG exporters."""

from __future__ import annotations

from .types import ElectricalObstacleMap, ElectricalPortAccess


def _all_port_accesses(
    obstacle_map: ElectricalObstacleMap,
) -> tuple[ElectricalPortAccess, ...]:
    accesses = [
        *obstacle_map.common_bus_port_accesses.values(),
        *obstacle_map.individual_port_accesses.values(),
    ]
    return tuple(
        sorted(
            accesses,
            key=lambda access: (access.purpose, access.terminal_id),
        )
    )
