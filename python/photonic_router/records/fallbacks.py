"""The router's fallbacks by name, and how often a run took each.

A fallback is a second way the router takes when the first one did not
work: a wider search, a simpler geometry, the standard router. Each has a
name here, and the place that takes it calls `take` with that name, so a
run can say which ones it needed. `scripts/logic_check.py` reports the
counts per benchmark as notes (`fallback.<name>`); a fallback that no
benchmark and no test ever takes is counted by the quality ratchet
(`docs/CODING_STANDARDS.md` rule 3.8).

The counts are per process: the flow calls `reset` when a run starts.
"""

from __future__ import annotations

from collections import Counter

#: name -> what is taken instead of what
FALLBACKS: dict[str, str] = {
    "flow.standard_router": (
        "pre-placed crossing grids were asked for, the design is not band-layered, "
        "and the standard router routes it"
    ),
    "crossings.layer_to_router": (
        "a layer with no feasible crossing structure keeps its nets for the router (per net)"
    ),
    "routing.full_grid_search": (
        "the search windows found no route and the search ran on the full grid (per route)"
    ),
    "routing.final_crossing_repair": (
        "illegal realized crossings were left after routing and a repair round "
        "ripped up and rerouted (per round)"
    ),
    "endpoint.checked_segment_rejected": (
        "the kernel rejected the checked correction of a terminal segment; "
        "that end keeps its baseline"
    ),
    "endpoint.length_along_straight_pieces": (
        "the kernel could not measure a corrected centerline; its length is the sum "
        "of its straight pieces"
    ),
    "realization.unchecked_port_centerline": (
        "a record reached realization without a corrected centerline and got "
        "the unchecked correction"
    ),
    "realization.grid_route_polygon": (
        "a record without a corrected centerline is drawn as its grid route"
    ),
    "plm.meander_split": (
        "no single meander fitted and the extra length was split over several runs of the same edge"
    ),
    "plm.meander_per_edge": (
        "the final meander sequence did not plan as a whole and each edge was planned alone"
    ),
    "plm.planner_error_as_no_candidate": (
        "the meander planner raised and the requirement counts as having no candidate"
    ),
    "plm.unregistered_route": (
        "a route without a centerline is not registered with the meander planner "
        "and stands as an obstacle"
    ),
    "electrical.pad_wire_full_grid": (
        "the channel router could not place a pad wire and the full-grid search "
        "routed it (per wire)"
    ),
}

_taken: Counter[str] = Counter()


def take(name: str, count: int = 1) -> None:
    """Record that the fallback `name` was taken `count` times.

    Raises:
        KeyError: `name` is not in `FALLBACKS`; a fallback is named there first.
    """
    if name not in FALLBACKS:
        raise KeyError(f"unknown fallback {name!r}; name it in FALLBACKS first")
    _taken[name] += count


def taken() -> dict[str, int]:
    """How often each fallback was taken since the last `reset`, sorted by name."""
    return dict(sorted(_taken.items()))


def reset() -> None:
    """Forget the counts; the flow calls this when a run starts."""
    _taken.clear()
