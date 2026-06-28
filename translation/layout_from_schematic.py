"""Convert gdsfactory.Schematic to unrouted gdsfactory.Component."""

import gdsfactory as gf
from gdsfactory.component import Component
from gdsfactory.schematic import Schematic
from uuid import uuid4


def _anchor_point_from_bbox(bbox, anchor: str) -> tuple[float, float]:
    """Return a dbbox anchor point using gdsfactory placement anchor names."""
    left = float(bbox.left)
    right = float(bbox.right)
    bottom = float(bbox.bottom)
    top = float(bbox.top)
    center_x = 0.5 * (left + right)
    center_y = 0.5 * (bottom + top)
    match anchor:
        case "sw":
            return left, bottom
        case "se":
            return right, bottom
        case "nw":
            return left, top
        case "ne":
            return right, top
        case "sc":
            return center_x, bottom
        case "nc":
            return center_x, top
        case "cw":
            return left, center_y
        case "ce":
            return right, center_y
        case "center" | "cc":
            return center_x, center_y
        case _:
            raise ValueError(f"Unsupported placement anchor '{anchor}'")


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
    component = gf.Component(f"unrouted_layout_{uuid4().hex}")

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
        anchor = placement.port

        # Add the component to the layout with transformation
        ref = component.add_ref(sub_component, name=instance_name)
        if anchor is None:
            ref.movex(x)
            ref.movey(y)
            ref.rotate(rotation)
            if mirror:
                ref.mirror()
        else:
            if mirror:
                ref.mirror()
            ref.rotate(rotation)
            anchor_x, anchor_y = _anchor_point_from_bbox(ref.dbbox(), str(anchor))
            ref.dmove((float(x) - anchor_x, float(y) - anchor_y))

    return component
