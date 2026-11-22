"""The checks of every stage a run reports, and the verifier that runs them in the flow.

A run reports its products through `routing.stage_observer`:
`stage_findings(stage, settings, product)` is the one place that says which
checks read which product. `scripts/logic_check.py` collects the findings
of a benchmark through it; `verify_stages()` runs it inside a normal run of
the flow (`--verify-stages`) and ends the run on the first stage with an
error.

Stages, in the order a run reports them: `loaded` (the schematic and the
placed layout; the reading checks and the crossing demand), `obstacle_map`,
`router_setup` (no checks yet), `handoff` (route jobs and crossing plan
info), `routed`, `records`, `verified`, `matched` (only a run with
path-length matching reports it), `realized`.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Any

from photonic_router.logic_checks.crossing_plan import (
    check_crossing_demand,
    check_crossing_plan_info,
)
from photonic_router.logic_checks.findings import ERROR, Finding
from photonic_router.logic_checks.matching import check_matched
from photonic_router.logic_checks.obstacle_map import check_obstacle_map
from photonic_router.logic_checks.reading import check_layout, check_schematic
from photonic_router.logic_checks.realized import check_realized, check_verified
from photonic_router.logic_checks.records import check_records
from photonic_router.logic_checks.route_jobs import check_route_jobs
from photonic_router.logic_checks.routing import check_routing
from photonic_router.placement_analysis import analyze_placement
from photonic_router.routing.stage_observer import observe_stages

#: the stages a run reports, in order
STAGES = (
    "loaded",
    "obstacle_map",
    "router_setup",
    "handoff",
    "routed",
    "records",
    "verified",
    "matched",
    "realized",
)


class StageVerificationError(RuntimeError):
    """A stage handed on a product that contradicts itself (`verify_stages`)."""

    def __init__(self, stage: str, errors: list[Finding]) -> None:
        self.stage = stage
        self.errors = errors
        shown = "; ".join(str(finding) for finding in errors[:5])
        more = f" (and {len(errors) - 5} more)" if len(errors) > 5 else ""
        super().__init__(f"stage {stage!r}: {len(errors)} error(s): {shown}{more}")


def stage_findings(stage: str, settings: Any, product: Any) -> list[Finding]:
    """The findings of the checks that read the product of `stage`.

    `settings` is the `SessionSettings` of the session that reports, None
    for the stages the flow reports itself (`loaded`, `matched`, `realized`).
    """
    if stage == "loaded":
        schematic, layout = product.schematic, product.unrouted_layout
        demand = analyze_placement(schematic, layout).crossing_demand
        return [
            *check_schematic(schematic),
            *check_layout(schematic, layout),
            *check_crossing_demand(schematic, layout, demand),
        ]
    if stage == "matched":
        return check_matched(product)
    if stage == "realized":
        return check_realized(product)
    if settings is None:
        return []
    if stage == "obstacle_map":
        return check_obstacle_map(
            settings.schematic, product.obstacle_map, product.resolved_obstacle_config
        )
    if stage == "handoff":
        analysis = settings.placement_analysis or analyze_placement(
            settings.schematic, settings.unrouted_layout
        )
        return [
            *check_route_jobs(settings.schematic, product),
            *check_crossing_plan_info(settings, product, analysis.crossing_demand),
        ]
    if stage == "routed":
        return check_routing(product)
    if stage == "records":
        return check_records(settings.schematic, product.routed_net_records)
    if stage == "verified":
        return check_verified(product)
    return []


@contextmanager
def verify_stages(report: Callable[[str], None] = print) -> Iterator[list[Finding]]:
    """Check every stage the runs inside the block report; an error ends the run.

    Yields the list the findings are collected in. After each stage one
    line goes to `report`; a stage with an error raises
    `StageVerificationError` from the phase that reported it. A session
    that runs more than once (contribution 2's probe session) is checked
    each time.
    """
    findings: list[Finding] = []

    def observer(stage: str, settings: Any, product: Any) -> None:
        found = stage_findings(stage, settings, product)
        findings.extend(found)
        errors = [finding for finding in found if finding.severity == ERROR]
        report(
            f"      - verify {stage}: {len(errors)} error(s), {len(found) - len(errors)} note(s)"
        )
        if errors:
            raise StageVerificationError(stage, errors)

    with observe_stages(observer):
        yield findings
