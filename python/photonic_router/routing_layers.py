"""Routing-layer and component-port access configuration helpers."""

from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatchcase
from typing import Iterable

Layer = tuple[int, int]

OPTICAL_OBSTACLE_LAYERS: tuple[Layer, ...] = ((1, 0),)

HEATER_METAL_OBSTACLE_LAYERS: tuple[Layer, ...] = (
    (47, 0),  # HEATER
    (45, 0),  # M2
    (49, 0),  # M3 / MTOP
    (44, 0),  # VIA1
    (43, 0),  # VIA2
)

HEATER_OPTICAL_ACCESS_LENGTH_UM = 20.0
HEATER_OPTICAL_ACCESS_WIDTH_UM = 12.0


@dataclass(frozen=True)
class ComponentPortAccessRule:
    """Directional port-access policy for selected component ports.

    The rule only describes how large an access opening should be for a
    matching port. The router still decides per net which source/target port
    openings are passed as temporary ``opened_cells``.
    """

    component_name_pattern: str
    port_names: tuple[str, ...]
    access_length_um: float
    access_width_um: float
    port_types: tuple[str, ...] = ("optical",)

    def matches(
        self,
        *,
        component_name: str | None,
        port_name: str | None,
        port_type: str | None = None,
    ) -> bool:
        if not component_name or not port_name:
            return False
        if not fnmatchcase(component_name, self.component_name_pattern):
            return False
        if port_name not in self.port_names:
            return False
        if port_type is None:
            return True
        return port_type in self.port_types


HEATER_OPTICAL_PORT_ACCESS_RULES: tuple[ComponentPortAccessRule, ...] = (
    ComponentPortAccessRule(
        component_name_pattern="straight_heater_metal*",
        port_names=("o1", "o2"),
        access_length_um=HEATER_OPTICAL_ACCESS_LENGTH_UM,
        access_width_um=HEATER_OPTICAL_ACCESS_WIDTH_UM,
    ),
)


def get_routing_obstacle_layers(
    *,
    include_heaters: bool = False,
) -> tuple[Layer, ...]:
    """Return the static obstacle layers used for optical routing."""

    layers: list[Layer] = list(OPTICAL_OBSTACLE_LAYERS)
    if include_heaters:
        layers.extend(HEATER_METAL_OBSTACLE_LAYERS)
    return _dedupe_layers(layers)


def find_component_port_access_rule(
    *,
    component_name: str | None,
    port_name: str | None,
    port_type: str | None = None,
    rules: Iterable[ComponentPortAccessRule] = HEATER_OPTICAL_PORT_ACCESS_RULES,
) -> ComponentPortAccessRule | None:
    """Return the first access rule matching a component port."""

    for rule in rules:
        if rule.matches(
            component_name=component_name,
            port_name=port_name,
            port_type=port_type,
        ):
            return rule
    return None


def _dedupe_layers(layers: Iterable[Layer]) -> tuple[Layer, ...]:
    seen: set[Layer] = set()
    ordered: list[Layer] = []
    for layer, datatype in layers:
        normalized = (int(layer), int(datatype))
        if normalized in seen:
            continue
        seen.add(normalized)
        ordered.append(normalized)
    return tuple(ordered)
