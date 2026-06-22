"""Extract logical two-terminal heater groups from placed gdsfactory instances."""

from __future__ import annotations

from collections import defaultdict
from fnmatch import fnmatchcase
from typing import Any, Iterable

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from .types import ElectricalPortRef, ElectricalRoutingConfig, ElectricalTerminal, TerminalPairGroup


def extract_heater_terminal_pairs(
    component: Component,
    schematic: Schematic | None = None,
    config: ElectricalRoutingConfig | None = None,
) -> tuple[TerminalPairGroup, ...]:
    """Return one two-terminal group for every heater instance.

    gdsfactory's ``straight_heater_metal`` exposes several electrical ports per
    side (for example ``l_e1``...``l_e4`` and ``r_e1``...``r_e4``). The router
    needs the logical terminal side, not every physical access port separately,
    so electrical ports are grouped into two side terminals per heater.
    """

    config = config or ElectricalRoutingConfig()
    config.validate()
    schematic_components = _schematic_component_names(schematic)

    groups: list[TerminalPairGroup] = []
    for instance in sorted(getattr(component, "insts", []), key=lambda inst: str(inst.name)):
        instance_name = str(getattr(instance, "name", "") or "")
        component_name = schematic_components.get(instance_name) or _instance_component_name(instance)
        if not _is_heater_instance(instance_name, component_name, config):
            continue

        electrical_ports = [_port_ref(port) for port in instance.ports if _is_electrical_port(port)]
        if not electrical_ports:
            raise ValueError(
                f"Heater instance '{instance_name}' has no electrical ports; "
                "cannot build terminal pair"
            )

        terminal_groups = _group_ports_into_two_terminals(instance_name, electrical_ports)
        if len(terminal_groups) != 2:
            keys = sorted(terminal_groups)
            raise ValueError(
                f"Heater instance '{instance_name}' must resolve to exactly two logical "
                f"electrical terminals, got {len(terminal_groups)} groups: {keys}"
            )

        terminals = [
            _make_terminal(instance_name, side_key, tuple(ports))
            for side_key, ports in sorted(
                terminal_groups.items(),
                key=lambda item: (_ports_center(item[1])[0], item[0]),
            )
        ]
        groups.append(
            TerminalPairGroup(
                heater_id=instance_name,
                terminal_a=terminals[0],
                terminal_b=terminals[1],
            )
        )

    return tuple(groups)


def _schematic_component_names(schematic: Schematic | None) -> dict[str, str]:
    if schematic is None:
        return {}
    instances = getattr(getattr(schematic, "netlist", None), "instances", {}) or {}
    return {
        str(instance_name): str(getattr(instance, "component", "") or "")
        for instance_name, instance in instances.items()
    }


def _instance_component_name(instance: Any) -> str:
    cell = getattr(instance, "cell", None)
    if cell is not None:
        return str(getattr(cell, "name", "") or "")
    return str(getattr(instance, "cell_name", "") or "")


def _is_heater_instance(
    instance_name: str,
    component_name: str,
    config: ElectricalRoutingConfig,
) -> bool:
    if any(instance_name.startswith(prefix) for prefix in config.heater_instance_prefixes):
        return True
    return any(
        fnmatchcase(component_name, pattern)
        for pattern in config.heater_component_patterns
    )


def _is_electrical_port(port: Any) -> bool:
    port_type = str(getattr(port, "port_type", "") or "").lower()
    if port_type == "electrical":
        return True
    name = str(getattr(port, "name", "") or "").lower()
    return "_e" in name or name.startswith("e")


def _port_ref(port: Any) -> ElectricalPortRef:
    center = getattr(port, "dcenter", None)
    if center is None:
        center = getattr(port, "center")
    return ElectricalPortRef(
        name=str(getattr(port, "name", "") or ""),
        center=(float(center[0]), float(center[1])),
        orientation=_optional_float(getattr(port, "orientation", None)),
        width=_optional_float(getattr(port, "width", None)),
        layer=getattr(port, "layer", None),
    )


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _group_ports_into_two_terminals(
    heater_id: str,
    ports: Iterable[ElectricalPortRef],
) -> dict[str, list[ElectricalPortRef]]:
    by_prefix: dict[str, list[ElectricalPortRef]] = defaultdict(list)
    ports = tuple(ports)
    for port in ports:
        prefix = _side_prefix(port.name)
        if prefix:
            by_prefix[prefix].append(port)

    if len(by_prefix) == 2:
        return dict(by_prefix)

    # Fallback for components that do not use l_*/r_* style port names: split
    # the electrical ports into left/right clusters by x coordinate.
    sorted_ports = sorted(ports, key=lambda port: (port.center[0], port.center[1], port.name))
    if len(sorted_ports) < 2:
        raise ValueError(f"Heater instance '{heater_id}' needs at least two electrical ports")
    midpoint = len(sorted_ports) // 2
    return {
        "left": sorted_ports[:midpoint],
        "right": sorted_ports[midpoint:],
    }


def _side_prefix(port_name: str) -> str | None:
    name = port_name.strip().lower()
    if not name:
        return None
    if "_" in name:
        return name.split("_", 1)[0]
    if name[0] in {"l", "r"}:
        return name[0]
    return None


def _make_terminal(
    heater_id: str,
    side_key: str,
    ports: tuple[ElectricalPortRef, ...],
) -> ElectricalTerminal:
    center = _ports_center(ports)
    bbox = _ports_bbox(ports)
    layer = ports[0].layer if ports else None
    return ElectricalTerminal(
        id=f"{heater_id}:{side_key}",
        heater_id=heater_id,
        side_key=side_key,
        center=center,
        bbox=bbox,
        ports=ports,
        layer=layer,
    )


def _ports_center(ports: Iterable[ElectricalPortRef]) -> tuple[float, float]:
    ports = tuple(ports)
    if not ports:
        raise ValueError("cannot compute center of empty port group")
    return (
        sum(port.center[0] for port in ports) / len(ports),
        sum(port.center[1] for port in ports) / len(ports),
    )


def _ports_bbox(ports: Iterable[ElectricalPortRef]) -> tuple[float, float, float, float]:
    ports = tuple(ports)
    if not ports:
        raise ValueError("cannot compute bbox of empty port group")
    xs_min: list[float] = []
    ys_min: list[float] = []
    xs_max: list[float] = []
    ys_max: list[float] = []
    for port in ports:
        half_width = max(float(port.width or 0.0) / 2.0, 0.0)
        xs_min.append(port.center[0] - half_width)
        xs_max.append(port.center[0] + half_width)
        ys_min.append(port.center[1] - half_width)
        ys_max.append(port.center[1] + half_width)
    return (min(xs_min), min(ys_min), max(xs_max), max(ys_max))
