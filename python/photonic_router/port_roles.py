"""Which optical ports of a schematic's instances light enters by, and which it leaves by.

The path-length graph (`path_length_graph.py`) is directed: a node's inputs
must arrive together, so every net has to know which of its two ends is
upstream. A schematic's nets carry no such direction -- a `Net` is a pair of
ports in whatever order its author wrote them, and the generated private
circuits write them in extraction order -- so the direction is read off the
generic components themselves. By the gdsfactory convention an input port
faces west in the component's own frame (orientation 180 degrees) and an
output port east (0 degrees). A component with a single optical port, a
grating or edge coupler, has no role of its own: its net takes the
direction from the other end.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

INPUT = "input"
OUTPUT = "output"

#: ``(instance, port) -> "input" | "output" | None``
PortRole = Callable[[str, str], str | None]


def no_port_roles(instance: str, port: str) -> str | None:
    """The resolver that knows nothing: every net keeps its declared direction."""

    return None


def generic_port_roles(schematic: Any) -> PortRole:
    """The port roles of every instance of ``schematic``, read from its generic components.

    Each distinct component and settings is built once through gdsfactory;
    a component that cannot be built (no active PDK, an unknown name) gives
    its instances no roles.
    """

    roles_by_instance: dict[str, dict[str, str]] = {}
    cache: dict[tuple[str, tuple[tuple[str, str], ...]], dict[str, str]] = {}
    instances = getattr(getattr(schematic, "netlist", None), "instances", None) or {}
    for name, instance in instances.items():
        component = getattr(instance, "component", None)
        if not isinstance(component, str):
            continue
        settings = dict(getattr(instance, "settings", None) or {})
        key = (component, tuple(sorted((k, repr(v)) for k, v in settings.items())))
        if key not in cache:
            cache[key] = component_port_roles(component, settings)
        roles_by_instance[str(name)] = cache[key]

    def role(instance: str, port: str) -> str | None:
        return roles_by_instance.get(instance, {}).get(port)

    return role


def component_port_roles(component: str, settings: dict[str, Any]) -> dict[str, str]:
    """``port name -> role`` for one generic component; empty for a one-port component."""

    try:
        import gdsfactory as gf

        built = gf.get_component(component, **settings)
        optical = [port for port in built.ports if port.port_type == "optical"]
    except Exception:
        return {}
    if len(optical) < 2:
        return {}
    roles: dict[str, str] = {}
    for port in optical:
        orientation = float(port.orientation) % 360.0
        if abs(orientation - 180.0) < 1e-6:
            roles[str(port.name)] = INPUT
        elif orientation < 1e-6 or abs(orientation - 360.0) < 1e-6:
            roles[str(port.name)] = OUTPUT
    return roles
