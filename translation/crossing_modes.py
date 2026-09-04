"""Crossing-mode vocabulary shared by the flow, the router session and the verifiers.

Three switchable configurations exist (owner rule, 2026-09-04; see
`.agent/execplans/2026-09-04-crossing-guided-search.md`): the baseline
``lidar-pure``, contribution 1 ``lidar-guided`` (same router-discovered
crossing mechanics, plus the topology plan as soft search guidance) and
contribution 2 (pre-placed crossing structures, a flow flag rather than a
crossing mode). Every site that asks "is this the router-discovered crossing
path?" must use :func:`is_lidar_mode` so that ``lidar-guided`` inherits the
lidar-pure mechanics exactly; only :func:`is_guided_mode` sites may consult
the plan.
"""

from __future__ import annotations

CROSSING_MODE_ALIASES: dict[str, str] = {
    "pure": "lidar-pure",
    "lidar": "lidar-pure",
    "guided": "lidar-guided",
    "lidar_guided": "lidar-guided",
}

LIDAR_MODES: frozenset[str] = frozenset({"lidar-pure", "lidar-guided"})
COLLISION_MODES: frozenset[str] = frozenset({"collision", *LIDAR_MODES})
CROSSING_MODES: tuple[str, ...] = ("window", "collision", "lidar-pure", "lidar-guided")


def normalize_crossing_mode(crossing_mode: object) -> str:
    """Lower-case, strip and resolve aliases; raises on unknown modes."""

    mode = str(crossing_mode).strip().lower()
    mode = CROSSING_MODE_ALIASES.get(mode, mode)
    if mode not in CROSSING_MODES:
        raise ValueError(
            "crossing_mode must be one of "
            + ", ".join(repr(name) for name in CROSSING_MODES)
            + f", got {crossing_mode!r}"
        )
    return mode


def is_lidar_mode(crossing_mode: object) -> bool:
    """Router-discovered crossings (lidar-pure mechanics), guided or not."""

    return str(crossing_mode).strip().lower() in LIDAR_MODES


def is_guided_mode(crossing_mode: object) -> bool:
    """Contribution 1: lidar mechanics plus the topology plan as guidance."""

    return str(crossing_mode).strip().lower() == "lidar-guided"


def is_collision_mode(crossing_mode: object) -> bool:
    """Any mode whose crossings are discovered by collision during the search."""

    return str(crossing_mode).strip().lower() in COLLISION_MODES
