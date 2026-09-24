"""Eleven heaters, two of them one row apart: the suite's default electrical case.

This is the default case of the electrical benchmark suite
(`scripts/benchmark_electrical.py::DEFAULT_BENCHMARK`): the four lanes' heater
pairs plus three extras staggered in x and y, so the individual escapes group
into several topology bundles of which one (four tracks) is the widest -- the
arrangement `test_auto_pad_channel_height_uses_widest_topology_bundle` compares
against the eleven-track channel a global assignment would need.

It replaces the deleted toy module `mmi_heater_8x4_ripup_reroute` (Milestone 8,
Slice D of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`) with one
heater moved: in the deleted toy, `heater_extra_1` and `heater_extra_2` sat in
one vertical corridor, which left their escapes 9.5 um apart, half a micron
inside the 10 um cross-net clearance, and the verifier rejected the result. Here
`heater_extra_2` sits one row lower (y 0.0 instead of 40.0), so the metrics of
this case are its own and not the toy's.

The optical part is trivial because there is none: this benchmark places heaters
only -- no MMIs, no grating couplers, no optical nets -- on the die outline of
`RIPUP_REROUTE_HEATER_DIE_UM`, which is drawn on the waveguide layer and so sets
the die extent without blocking any electrical route (see
`benchmarks/heater_lanes_20.py`, whose `heater_only_schematic` this module
reuses).
"""

from gdsfactory.schematic import Schematic

from benchmarks.heater_lanes_20 import heater_only_schematic

RIPUP_REROUTE_HEATER_PLACEMENTS_UM: dict[str, tuple[float, float]] = {
    "heater_0": (300.0, 280.0),
    "heater_post_0": (820.0, 280.0),
    "heater_1": (300.0, 120.0),
    "heater_post_1": (820.0, 120.0),
    "heater_2": (300.0, -40.0),
    "heater_post_2": (820.0, -40.0),
    "heater_3": (300.0, -200.0),
    "heater_post_3": (820.0, -200.0),
    "heater_extra_0": (1480.0, 180.0),
    "heater_extra_1": (2340.0, 40.0),
    "heater_extra_2": (3000.0, 0.0),
}

# 600 um tall for the same reason as `MULTI_HEATER_DIE_UM`, 3.75 mm wide because
# the heaters end at x 3000.
RIPUP_REROUTE_HEATER_DIE_UM = (-50.0, -300.0, 3750.0, 600.0)

NODE_TYPES = dict.fromkeys(RIPUP_REROUTE_HEATER_PLACEMENTS_UM, "gate")

# Resolved from straight_heater_metal.info.length at metadata load time. The die
# outline is not a routing node and carries no optical path, so it is in neither
# table.
INTERNAL_DELAYS_UM: dict[str, object] = dict.fromkeys(RIPUP_REROUTE_HEATER_PLACEMENTS_UM, "auto")


def build_schematic() -> Schematic:
    """Eleven heaters, two of them one row apart; see
    `RIPUP_REROUTE_HEATER_PLACEMENTS_UM`."""

    return heater_only_schematic(RIPUP_REROUTE_HEATER_PLACEMENTS_UM, RIPUP_REROUTE_HEATER_DIE_UM)
