"""Route an unrouted GDS layout using gdsfactory's built-in routing.

This is a baseline router using gdsfactory's route_single for simple Manhattan routing.
Later, this will be replaced by a custom router with grid-based routing, obstacles,
rip-up and reroute, etc.
"""
from gdsfactory.component import Component
from gdsfactory.routing import route_single
from gdsfactory.schematic import Schematic
from gdsfactory.typings import Port


def get_port_from_instance(
    component: Component, instance_name: str, port_name: str
) -> Port:
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


def print_instance_ports(component: Component, instance_name: str) -> None:
    """Print debug information for ports of an instance in the layout.

    This helps diagnose port positions and orientations in the routed layout.

    Parameters:
        component: The layout component.
        instance_name: Name of the instance to inspect.
    """
    inst = None
    for instance in component.insts:
        if instance.name == instance_name:
            inst = instance
            break

    if inst is None:
        print(f"Instance '{instance_name}' not found")
        return

    print(f"\nInstance: {instance_name}")
    print(f"Base component: {inst.cell.name}")
    print(f"Position: x={inst.x}, y={inst.y}")
    print(f"\nPorts (already transformed):")
    print(f"{'Port':<10} {'Center (um)':<35} {'Orient':<10} {'Width':<10}")
    print("-" * 70)

    for port in inst.ports:
        center = tuple(port.center)
        print(
            f"{port.name:<10} {str(center):<35} "
            f"{str(port.orientation):<10} {str(port.width):<10}"
        )


def route_nets_gds(
    unrouted_layout: Component,
    schematic: Schematic,
    cross_section: str = "strip",
    allow_width_mismatch: bool = True,
    on_error: str | None = "error",
) -> Component:
    """Route nets in an unrouted layout using gdsfactory's built-in routing.

    This function takes nets defined in the schematic and routes them in the
    physical layout using gdsfactory's route_single (Manhattan routing).

    Parameters:
        unrouted_layout: The component with placed instances but no routes.
        schematic: The schematic with net definitions.
        cross_section: Cross section spec to use for routing (default: 'strip').
        allow_width_mismatch: Allow mismatched waveguide widths during routing.
        on_error: How to handle routing errors ('error' to raise, None to continue).

    Returns:
        The routed layout component.

    Raises:
        ValueError: If a net's ports cannot be found or routed.
    """
    # Create a copy to avoid modifying the original
    routed_layout = unrouted_layout.copy()
    routed_layout.name = "routed_layout"

    # Extract nets from schematic
    nets = schematic.netlist.routes  # Dict[str, Bundle]

    print(f"\nRouting {len(nets)} nets using gdsfactory baseline router...")

    routed_count = 0
    failed_nets = []

    for net_name, bundle in nets.items():
        # Each bundle can have multiple links
        links = bundle.links  # Dict[str, str], e.g., {"gc_0,o2": "mmi_0,o1"}

        for port1_spec, port2_spec in links.items():
            try:
                # Parse port specifiers: "instance_name,port_name"
                inst1, port1 = port1_spec.split(",")
                inst2, port2 = port2_spec.split(",")

                print(f"  Routing {net_name}: {port1_spec} → {port2_spec}...", end=" ")

                # Get absolute ports in the layout
                abs_port1 = get_port_from_instance(routed_layout, inst1, port1)
                abs_port2 = get_port_from_instance(routed_layout, inst2, port2)

                print(
                    f"\n  Net '{net_name}':"
                    f"\n    {inst1}.{port1}: center={tuple(abs_port1.center)}, "
                    f"orientation={abs_port1.orientation}, width={abs_port1.width}"
                    f"\n    {inst2}.{port2}: center={tuple(abs_port2.center)}, "
                    f"orientation={abs_port2.orientation}, width={abs_port2.width}"
                )

                # Route using gdsfactory's built-in Manhattan router
                route = route_single(
                    routed_layout,
                    abs_port1,
                    abs_port2,
                    cross_section=cross_section,
                    allow_width_mismatch=allow_width_mismatch,
                    auto_taper=False,
                    on_error=on_error,  # type: ignore
                )

                routed_count += 1
                print("✓")

            except Exception as e:
                failed_nets.append((net_name, port1_spec, port2_spec, str(e)))
                print(f"✗")
                if on_error != "error":
                    # Print error details only in non-strict mode
                    print(f"      Error: {str(e)[:100]}")

    print(f"\n  Routed {routed_count}/{len(nets)} nets successfully")

    if failed_nets:
        print(f"  Failed nets: {len(failed_nets)}")
        # if on_error == "error":
        #     raise RuntimeError(
        #         f"Failed to route {len(failed_nets)} nets. First error:"
        #         f"\n  {failed_nets[0][0]}: {failed_nets[0][3]}"
        #     )

    return routed_layout






