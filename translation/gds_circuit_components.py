"""Generic components the rebuilt circuits need beyond the generic PDK's catalogue.

Private-circuits plan, 2026-10-06 (owner's review of the beam-splitter
circuit): an MZI arm pair -- two waveguides that enter as a parallel pair,
fan out over S-bends to two straight arms and rejoin to a parallel pair --
is a given structure of the design, one component with two inputs and two
outputs, not four nets to route. ``arm_pair`` builds it from straights and
circular 90-degree bends, the router's own primitives.

Same day, the owner's review of the second Clements mesh: the generic PDK has
no two-port (polarization-splitting) grating coupler whose ports leave side
by side, and modelling one as two single-port couplers at the original 9.9 um
pitch made the two overlap -- elements must never overlap.
``grating_coupler_2d`` is that device as one square component with two
ports.

``register_generic_components`` makes both available under their names in
the generic PDK so that schematics and the generated benchmark modules can
name them like any other cell.
"""

from __future__ import annotations

import gdsfactory as gf

ARM_PAIR = "arm_pair"
GRATING_COUPLER_2D = "grating_coupler_2d"


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


@gf.cell
def grating_coupler_2d(
    port_pitch: float = 9.9,
    taper_length: float = 10.0,
    taper_width: float = 4.0,
    grating_size: float = 20.0,
    grating_period: float = 0.6,
    cross_section: str = "strip",
) -> gf.Component:
    """A two-port grating coupler as one device with its two ports side by side.

    ``o1`` (lower) and ``o2`` (upper) sit on the left at x = 0, ``port_pitch``
    apart, facing left like ``grating_coupler_te``'s ``o1``. Each widens over
    a linear taper of ``taper_length`` to ``taper_width`` into a square slab
    of ``grating_size`` on the core layer, whose grating is drawn as lines of
    ``grating_period`` on the shallow-etch layer: one slab shared by both
    tapers when the ports sit closer than the slab is wide (the 2D coupler of
    the Clements meshes, 9.9 um), otherwise one slab per port (the beam
    splitter's coupler cell, whose two ports are 482 um apart). The whole
    body lies to the right of the port plane: a route leaving a port meets
    nothing but its own taper, and two couplers of a circuit at their
    original pitch never overlap each other.
    """

    if taper_width >= port_pitch:
        raise ValueError(f"taper width {taper_width} leaves no gap at a port pitch of {port_pitch}")
    component = gf.Component()
    width = gf.get_cross_section(cross_section).width
    shared = port_pitch + taper_width <= grating_size
    for name, sign in (("o1", -1.0), ("o2", 1.0)):
        taper = component << gf.components.taper(
            length=taper_length, width1=width, width2=taper_width, cross_section=cross_section
        )
        taper.move((0.0, sign * port_pitch / 2))
        component.add_port(name, port=taper.ports["o1"])
        if not shared:
            _grating_slab(
                component, taper_length, sign * port_pitch / 2, grating_size, grating_period
            )
    if shared:
        _grating_slab(component, taper_length, 0.0, grating_size, grating_period)
    return component


def _grating_slab(
    component: gf.Component, left: float, center_y: float, size: float, period: float
) -> None:
    """A ``size`` square slab on the core layer from ``left``; grating lines on the etch layer."""

    half = size / 2
    right = left + size
    bottom, top = center_y - half, center_y + half
    component.add_polygon([(left, bottom), (right, bottom), (right, top), (left, top)], layer="WG")
    x = left + period
    while x + period / 2 < right:
        component.add_polygon(
            [
                (x - period / 4, bottom),
                (x + period / 4, bottom),
                (x + period / 4, top),
                (x - period / 4, top),
            ],
            layer="SHALLOW_ETCH",
        )
        x += period


def register_generic_components() -> None:
    """Register the components above in the active generic PDK (idempotent)."""

    from gdsfactory.gpdk import get_generic_pdk

    pdk = get_generic_pdk()
    pdk.activate()
    for name, cell in ((ARM_PAIR, arm_pair), (GRATING_COUPLER_2D, grating_coupler_2d)):
        if name not in pdk.cells:
            pdk.register_cells(**{name: cell})
