"""Generic components the rebuilt circuits need beyond the generic PDK's catalogue.

Private-circuits plan, 2026-10-06 (owner's review of the beam-splitter
circuit): an MZI arm pair -- two waveguides that enter as a parallel pair,
fan out over S-bends to two straight arms and rejoin to a parallel pair --
is a given structure of the design, one component with two inputs and two
outputs, not four nets to route. ``arm_pair`` builds it from straights and
circular 90-degree bends, the router's own primitives;
``register_generic_components`` makes it
available under that name in the generic PDK so that schematics and the
generated benchmark modules can name it like any other cell.
"""

from __future__ import annotations

import gdsfactory as gf

ARM_PAIR = "arm_pair"


@gf.cell
def arm_pair(
    arm_length: float = 146.0,
    arm_separation: float = 58.0,
    extension: float = 60.0,
    pitch_in: float = 2.0,
    pitch_out: float = 2.0,
    bend_radius: float = 5.0,
    cross_section: str = "strip",
) -> gf.Component:
    """Two arms ``arm_separation`` apart, spread and rejoined with 90-degree bends.

    The same primitives the router draws: straights and circular 90-degree
    arcs of ``bend_radius``. Each arm leaves its input port along a straight
    of ``extension - 2 * bend_radius``, turns away from the axis, rises over a
    straight, turns back, runs ``arm_length``, and returns the same way, so
    the component spans ``2 * extension + arm_length`` like the S-bend
    original. Ports like an MMI2x2: ``o1`` lower and ``o2`` upper input on
    the left, ``pitch_in`` apart; ``o3`` upper and ``o4`` lower output on the
    right, ``pitch_out`` apart.
    """

    component = gf.Component()
    lead = extension - 2 * bend_radius
    if lead < 0:
        raise ValueError(f"extension {extension} shorter than two bends of {bend_radius}")
    for sign, in_name, out_name in ((-1.0, "o1", "o4"), (1.0, "o2", "o3")):
        rise_in = (arm_separation - pitch_in) / 2 - 2 * bend_radius
        rise_out = (arm_separation - pitch_out) / 2 - 2 * bend_radius
        if min(rise_in, rise_out) < 0:
            raise ValueError("arm separation too small for two bends on each side")
        path = (
            gf.path.straight(length=lead)
            + gf.path.arc(radius=bend_radius, angle=90 * sign)
            + gf.path.straight(length=rise_in)
            + gf.path.arc(radius=bend_radius, angle=-90 * sign)
            + gf.path.straight(length=arm_length)
            + gf.path.arc(radius=bend_radius, angle=-90 * sign)
            + gf.path.straight(length=rise_out)
            + gf.path.arc(radius=bend_radius, angle=90 * sign)
            + gf.path.straight(length=lead)
        )
        arm = component << gf.path.extrude(path, cross_section=cross_section)
        arm.move((0.0, sign * pitch_in / 2))
        component.add_port(in_name, port=arm.ports["o1"])
        component.add_port(out_name, port=arm.ports["o2"])
    return component


def register_generic_components() -> None:
    """Register the components above in the active generic PDK (idempotent)."""

    from gdsfactory.gpdk import get_generic_pdk

    pdk = get_generic_pdk()
    pdk.activate()
    if ARM_PAIR not in pdk.cells:
        pdk.register_cells(**{ARM_PAIR: arm_pair})
