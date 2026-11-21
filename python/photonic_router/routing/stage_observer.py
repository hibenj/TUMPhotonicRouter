"""A seam for looking at what a routing session's phases hand on.

`observe_stages(observer)` is a context manager; while it is open, every
session of the current context calls `observer(stage, settings, product)`
after each phase it reports (`notify`). Outside it `notify` does nothing,
so a normal run pays one lookup per phase.

The logic harness (`photonic_router.logic_checks`, `scripts/logic_check.py`)
is the reader: it checks the product of a phase and raises `StopAfterStage`
from the observer once it has seen the last phase it has checks for, which
ends the session without routing. The caller that opened `observe_stages`
catches it.

Stages reported so far, with their product:

- `"loaded"`: the `LoadedDesign` of the flow, the schematic and its placed
  layout before anything is routed; reported without session settings
- `"obstacle_map"`: phase 1's `ObstacleContext`
- `"router_setup"`: the `SessionState` after phase 2
- `"handoff"`: the `SessionState` after phase 5, before the dispatch
- `"routed"`: the `SessionState` after phase 6, the kernel batch: every net
  has its committed route, no end is corrected yet
- `"records"`: the `RoutedRecords` after phase 7 and the optional detour
  pass: the routed net records with their ends corrected to the ports
- `"verified"`: the `VerifiedRecords` after phase 8: the records after the
  repair loops and the realized crossings that are still illegal
- `"realized"`: the `RealizedNets` of `route_match_and_realize`, after the
  session, the path-length matching and the realization into polygons;
  reported without session settings (`None`)
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from contextvars import ContextVar
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from photonic_router.routing.settings import SessionSettings

StageObserver = Callable[[str, "SessionSettings | None", object], None]

_OBSERVER: ContextVar[StageObserver | None] = ContextVar("stage_observer", default=None)


class StopAfterStage(Exception):
    """Raised by an observer to end the session after the stage it was called for."""


@contextmanager
def observe_stages(observer: StageObserver) -> Iterator[None]:
    """Call `observer` after every reported phase of the sessions run inside the block."""
    token = _OBSERVER.set(observer)
    try:
        yield
    finally:
        _OBSERVER.reset(token)


def notify(stage: str, settings: SessionSettings | None, product: object) -> None:
    """Hand a phase's product to the observer of the current context, if there is one."""
    observer = _OBSERVER.get()
    if observer is not None:
        observer(stage, settings, product)
