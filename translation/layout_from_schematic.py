"""Convert gdsfactory.Schematic to unrouted gdsfactory.Component."""

import gdsfactory as gf
from gdsfactory.component import Component
from gdsfactory.schematic import Schematic


def layout_from_schematic(schematic: Schematic) -> Component:
    """Convert a schematic to an unrouted layout component.

    This function places all instances from the schematic into a new component
    without routing. The result is a component with placed but unrouted instances.

    Parameters:
        schematic: A gdsfactory Schematic object containing instances and placements.

    Returns:
        A gdsfactory Component with all instances placed but not routed.
    """
    # Create a new component to hold the layout
    component = gf.Component("unrouted_layout")

    # Iterate through instances and placements
    for instance_name, instance in schematic.netlist.instances.items():
        placement = schematic.placements.get(instance_name)

        if placement is None:
            raise ValueError(f"No placement found for instance '{instance_name}'")

        # Get the component specification from the instance
        component_name = instance.component
        component_settings = instance.settings

        # Retrieve the actual gdsfactory component
        sub_component = gf.get_component(component_name, settings=component_settings)

        # sub_component.pprint_ports()

        # Extract placement parameters
        x = placement.x if placement.x is not None else 0
        y = placement.y if placement.y is not None else 0
        rotation = placement.rotation if placement.rotation is not None else 0
        mirror = placement.mirror if placement.mirror is not None else False

        # Add the component to the layout with transformation
        ref = component.add_ref(sub_component, name=instance_name)
        ref.movex(x)
        ref.movey(y)
        ref.rotate(rotation)
        if mirror:
            ref.mirror()

    return component

