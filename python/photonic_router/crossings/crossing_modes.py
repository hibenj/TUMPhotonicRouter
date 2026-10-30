"""Crossing-mode vocabulary shared by the flow, the router session and the verifiers.

Two crossing modes exist (owner rule, 2026-09-04; see
`.agent/execplans/2026-09-04-crossing-guided-search.md`): the baseline
``lidar-pure`` and contribution 1 ``lidar-guided`` (same router-discovered
crossing mechanics, plus the topology plan as soft search guidance).
Contribution 2 (pre-placed crossing structures) is a flow flag rather than a
crossing mode. Both modes share the router-discovered crossing mechanics, so
the only question a site may ask is :func:`is_guided_mode` (may it consult
the plan?); every other crossing decision is keyed on ``enable_crossings``.

The older ``window`` and ``collision`` modes were removed on 2026-09-24
(Milestone 8 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`); both
names are rejected by :func:`normalize_crossing_mode`, as is every alias.
"""

from __future__ import annotations

CROSSING_MODES: tuple[str, ...] = ("lidar-pure", "lidar-guided")


def normalize_crossing_mode(crossing_mode: object) -> str:
    """Lower-case and strip; raises on unknown modes."""

    mode = str(crossing_mode).strip().lower()
    if mode not in CROSSING_MODES:
        raise ValueError(
            "crossing_mode must be one of "
            + ", ".join(repr(name) for name in CROSSING_MODES)
            + f", got {crossing_mode!r}"
        )
    return mode


def is_guided_mode(crossing_mode: object) -> bool:
    """Contribution 1: lidar mechanics plus the topology plan as guidance."""

    return str(crossing_mode).strip().lower() == "lidar-guided"
