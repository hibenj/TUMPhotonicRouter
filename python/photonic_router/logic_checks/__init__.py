"""The logic harness: does what each stage hands on make sense?

The lint stage checks form and the gates check end numbers. These checks
sit between: after a stage has run, its product is tested against
statements that have to hold for the next stage to mean anything (every
link names a port that exists, a port points out of its component, ...).

Each stage has a module here with pure functions from the stage's product
to a list of `Finding`. A finding is an *error* (the product contradicts
itself; the next stage would work on nonsense) or a *note* (a fact that is
legal but that a later stage has to handle, such as a port recessed from
its component's face). `stages.py` says which checks read which stage and
runs them inside a run of the flow (`--verify-stages`: an error ends the
run); `scripts/logic_check.py` runs them on benchmarks and `tests/logic/`
pins what they find.

Stages so far: `reading` (the schematic and the placed layout),
`crossing_plan` (the crossing demand of the placement analysis and what
phase 4 of the session takes of it), `obstacle_map` (phase 1 of the routing
session), `route_jobs` (the jobs and openings after phase 5), `routing`
(the routes the kernel hands back after phase 6), `records` (the routed
net records after the ends were corrected, phase 7), `matching` (the path
lengths once the meanders are planned, in the records and in the layout)
and `realized` (phase 8's product and the polygons of the routed layout);
the stages of a run
are read through `routing.stage_observer`.
"""

from __future__ import annotations

from photonic_router.logic_checks.findings import ERROR, NOTE, Finding, count_by_check

__all__ = ["ERROR", "NOTE", "Finding", "count_by_check"]
