"""Look up a port of a placed instance in a gdsfactory layout."""

from __future__ import annotations

from gdsfactory.component import Component
from gdsfactory.typings import Port


def get_port_from_instance(component: Component, instance_name: str, port_name: str) -> Port:
    """Extract a physical port from an instance in a component.

    The ports are already transformed (rotated, mirrored, translated) when accessed
    through the instance reference. This function simply retrieves the correct port.

    Parameters:
        component: The layout component containing instances.
        instance_name: Name of the instance (e.g., 'gc_0').
        port_name: Name of the port on that instance (e.g., 'o2').

    Returns:
        The already-transformed Port with correct coordinates and orientation.

    Raises:
        ValueError: If instance or port not found.
    """
    # Get the instance by name
    inst = None
    for instance in component.insts:
        if instance.name == instance_name:
            inst = instance
            break

    if inst is None:
        raise ValueError(f"Instance '{instance_name}' not found in component")

    # Get the transformed port from the instance
    # The ports accessed through inst.ports are already transformed
    # (position, rotation, mirror all applied)
    if port_name not in inst.ports:
        available = [p.name for p in inst.ports]
        raise ValueError(
            f"Port '{port_name}' not found in instance '{instance_name}'. "
            f"Available ports: {available}"
        )

    return inst.ports[port_name]
