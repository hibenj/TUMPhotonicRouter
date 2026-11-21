"""Run the logic harness on benchmarks: does what each stage hands on make sense?

Usage:
    logic_check.py <benchmark> [<benchmark> ...] [--list N]
    logic_check.py --all [--list N]
    logic_check.py --all --update

Runs the checks of `photonic_router.logic_checks` stage by stage and prints,
per benchmark, how many findings each check has; `--list N` also prints the
first N findings of each check. Exits 1 when a benchmark has an error
(with `--update`: writes the pin and exits 0; the test of the pin names the
errors that are known).

`--all` takes every module of `benchmarks/` with a `build_schematic()`
(not `benchmarks/private/`; name those modules as `private.<name>`).
`--update` writes the counts to `tests/baselines/logic_findings.json`, the
pin that `tests/logic/test_logic_findings.py` compares against; a pin that
moves is re-pinned with the reason in the commit.
"""

from __future__ import annotations

import argparse
import contextlib
import importlib
import io
import json
import pkgutil
import re
import sys
from pathlib import Path
from typing import cast

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from photonic_router import cli
from photonic_router.flow import load_benchmark
from photonic_router.logic_checks import ERROR, Finding, count_by_check
from photonic_router.logic_checks.crossing_plan import (
    check_crossing_demand,
    check_crossing_plan_info,
)
from photonic_router.logic_checks.obstacle_map import check_obstacle_map
from photonic_router.logic_checks.reading import check_layout, check_schematic
from photonic_router.logic_checks.records import check_records
from photonic_router.logic_checks.route_jobs import check_route_jobs
from photonic_router.logic_checks.routing import check_routing
from photonic_router.placement_analysis import analyze_placement
from photonic_router.records.layout_from_schematic import layout_from_schematic
from photonic_router.routing.products import ObstacleContext
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages
from photonic_router.routing.stages import RoutedRecords

PIN = REPO_ROOT / "tests" / "baselines" / "logic_findings.json"


def public_benchmarks() -> list[str]:
    """The modules of `benchmarks/` that build a schematic, sorted."""
    package = importlib.import_module("benchmarks")
    names = []
    for module in pkgutil.iter_modules(package.__path__):
        if module.ispkg:
            continue
        if hasattr(importlib.import_module(f"benchmarks.{module.name}"), "build_schematic"):
            names.append(module.name)
    return sorted(names)


#: the stages a session reports, in order (`routing.stage_observer`)
STAGES = ("obstacle_map", "router_setup", "handoff", "routed", "records")


def last_stage(benchmark: str) -> str:
    """The last stage a benchmark is run to: `records`, or `handoff` for the wide designs.

    From 16 x 16 on the search takes over a minute (Benes 32 x 32: eight),
    so those designs are checked up to the hand-off and not routed.
    """
    match = re.search(r"_(\d+)x\d+", benchmark)
    return "handoff" if match and int(match.group(1)) >= 16 else "records"


def findings_for_benchmark(benchmark: str, *, session: bool = True) -> list[Finding]:
    """Every finding of every stage implemented so far, in stage order.

    `session=False` checks the reading stage and the crossing demand only
    (the session stages take over a minute on the largest benchmarks).

    The reading stage and the crossing demand of the placement analysis are
    checked on the benchmark as loaded. The session stages are checked on a
    real run of the flow with the benchmark's stable defaults
    (`cli.main([benchmark])`), which is ended after `last_stage(benchmark)`:
    after the nets are routed and their ends corrected, or before the search.
    """
    schematic = load_benchmark(benchmark)
    findings = check_schematic(schematic)
    layout = layout_from_schematic(schematic)
    findings.extend(check_layout(schematic, layout))
    demand = analyze_placement(schematic, layout).crossing_demand
    findings.extend(check_crossing_demand(schematic, layout, demand))
    if session:
        findings.extend(_session_findings(benchmark, last_stage(benchmark)))
    return findings


def _session_findings(benchmark: str, last: str) -> list[Finding]:
    findings: list[Finding] = []
    seen: list[str] = []

    def observer(stage: str, settings: SessionSettings, product: object) -> None:
        seen.append(stage)
        if stage == "obstacle_map":
            context = cast(ObstacleContext, product)
            findings.extend(
                check_obstacle_map(
                    settings.schematic, context.obstacle_map, context.resolved_obstacle_config
                )
            )
        if stage == "handoff":
            findings.extend(check_route_jobs(settings.schematic, product))
            analysis = settings.placement_analysis or analyze_placement(
                settings.schematic, settings.unrouted_layout
            )
            findings.extend(check_crossing_plan_info(settings, product, analysis.crossing_demand))
        if stage == "routed":
            findings.extend(check_routing(product))
        if stage == "records":
            routed = cast(RoutedRecords, product)
            findings.extend(check_records(settings.schematic, routed.routed_net_records))
        if stage == last:
            raise StopAfterStage

    try:
        with (
            observe_stages(observer),
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            cli.main([benchmark])
    except StopAfterStage:
        return findings
    except RuntimeError as error:
        if "handoff" in seen and "routed" not in seen and "No route found" in str(error):
            # the kernel batch gave up on a net: the flow's own failure path
            findings.append(
                Finding("routing.net_not_routed", ERROR, benchmark, str(error).splitlines()[0])
            )
            return findings
        raise
    raise RuntimeError(f"{benchmark}: the flow ended without stage {last!r}; saw {seen}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("benchmarks", nargs="*")
    parser.add_argument("--all", action="store_true", help="every public benchmark")
    parser.add_argument("--list", type=int, default=0, metavar="N", help="print N findings each")
    parser.add_argument("--update", action="store_true", help="write the pin (needs --all)")
    args = parser.parse_args(argv)
    if args.update and not args.all:
        parser.error("--update needs --all")
    names = public_benchmarks() if args.all else args.benchmarks
    if not names:
        parser.error("name a benchmark or pass --all")

    counts: dict[str, dict[str, int]] = {}
    failed = False
    for name in names:
        findings = findings_for_benchmark(name)
        counts[name] = count_by_check(findings)
        errors = sum(finding.severity == ERROR for finding in findings)
        failed = failed or errors > 0
        summary = ", ".join(f"{check} {count}" for check, count in counts[name].items())
        print(f"{name}: {errors} error(s); {summary or 'no findings'}")
        shown: dict[str, int] = {}
        for finding in findings:
            if shown.get(finding.check, 0) < args.list:
                shown[finding.check] = shown.get(finding.check, 0) + 1
                print(f"    {finding}")
    if args.update:
        PIN.write_text(json.dumps(counts, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"pinned {len(counts)} benchmark(s) in {PIN.relative_to(REPO_ROOT)}")
        return 0
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
