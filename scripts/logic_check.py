"""Run the logic harness on benchmarks: does what each stage hands on make sense?

Usage:
    logic_check.py <benchmark> [<benchmark> ...] [--configuration NAME] [--list N]
    logic_check.py --all [--configuration NAME] [--list N]
    logic_check.py --all --update
    logic_check.py <benchmark> --route-wide

Runs the checks of `photonic_router.logic_checks` stage by stage, once per
router configuration (`baseline`: the standard router; `contribution1`:
crossings planned ahead and the search guided by the plan; `contribution2`:
pre-placed crossing structures), and prints per benchmark and configuration
how many findings each check has; `--list N` also prints the first N
findings of each check. `--configuration` (repeatable) limits the run to
the named ones. Exits 1 when a run has an error (with `--update`: writes
the pin and exits 0; the test of the pin names the errors that are known).

The wide designs (16 x 16 and up) are checked up to the hand-off and not
routed; `--route-wide` routes them too (minutes per design and
configuration; not part of the pin).

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
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from photonic_router import cli
from photonic_router.logic_checks import ERROR, NOTE, Finding, count_by_check
from photonic_router.logic_checks.stages import stage_findings
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.stage_observer import StopAfterStage, observe_stages

PIN = REPO_ROOT / "tests" / "baselines" / "logic_findings.json"

#: the three router configurations of the paper, as `--configuration` names them
CONFIGURATIONS = ("baseline", "contribution1", "contribution2")


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


def last_stage(benchmark: str, *, route_wide: bool = False) -> str:
    """The last stage a benchmark is run to: `realized`, or `handoff` for the wide designs.

    From 16 x 16 on the search takes over a minute (Benes 32 x 32: eight),
    so those designs are checked up to the hand-off and not routed unless
    `route_wide` is set.
    """
    match = re.search(r"_(\d+)x\d+", benchmark)
    wide = match is not None and int(match.group(1)) >= 16
    return "handoff" if wide and not route_wide else "realized"


def findings_for_benchmark(
    benchmark: str,
    *,
    configuration: str = "baseline",
    session: bool = True,
    route_wide: bool = False,
) -> list[Finding]:
    """Every finding of every stage, in the order a run reports the stages.

    The stages are checked on a real run of the flow with the benchmark's
    stable defaults under one of `CONFIGURATIONS`
    (`cli.main([benchmark, "--configuration", configuration])`), through
    `logic_checks.stages.stage_findings`, the checks `--verify-stages` runs.
    The run is ended after `last_stage(benchmark)`: after the routes are
    polygons, or before the search. `session=False` ends it after the stage
    `loaded` (the reading checks and the crossing demand; the session
    stages take over a minute on the largest benchmarks).
    """
    last = last_stage(benchmark, route_wide=route_wide) if session else "loaded"
    findings: list[Finding] = []
    seen: list[str] = []

    def observer(stage: str, settings: SessionSettings | None, product: object) -> None:
        seen.append(stage)
        findings.extend(stage_findings(stage, settings, product))
        if stage == last:
            raise StopAfterStage

    try:
        with (
            observe_stages(observer),
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            cli.main([benchmark, "--configuration", configuration])
    except StopAfterStage:
        return findings
    except (RuntimeError, ValueError) as error:
        findings.append(_ended_run(benchmark, seen, error))
        return findings
    raise RuntimeError(
        f"{benchmark} ({configuration}): the flow ended without stage {last!r}; saw {seen}"
    )


def _ended_run(benchmark: str, seen: list[str], error: Exception) -> Finding:
    """What a run that the flow ended with an exception is reported as.

    - `routing.net_not_routed` (error): the kernel batch gave up on a net
    - `flow.design_refused` (note): the configuration says it is not made
      for this design (pre-placed crossing grids need a band-layered one)
    - `flow.run_failed` (error): anything else
    """
    message = str(error).splitlines()[0][:300]
    if "handoff" in seen and "routed" not in seen and "No route found" in message:
        return Finding("routing.net_not_routed", ERROR, benchmark, message)
    if "need a band-layered design" in message:
        return Finding("flow.design_refused", NOTE, benchmark, message)
    return Finding("flow.run_failed", ERROR, benchmark, f"after {seen[-1:]}: {message}")


def _run(task: tuple[str, str, bool]) -> tuple[str, str, list[Finding]]:
    name, configuration, route_wide = task
    findings = findings_for_benchmark(name, configuration=configuration, route_wide=route_wide)
    return name, configuration, findings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("benchmarks", nargs="*")
    parser.add_argument("--all", action="store_true", help="every public benchmark")
    parser.add_argument("--list", type=int, default=0, metavar="N", help="print N findings each")
    parser.add_argument(
        "--configuration",
        action="append",
        choices=CONFIGURATIONS,
        help="a router configuration to run (repeatable; default: all three)",
    )
    parser.add_argument(
        "--route-wide", action="store_true", help="route the wide designs too (not for the pin)"
    )
    parser.add_argument("--jobs", type=int, default=1, metavar="N", help="runs in parallel")
    parser.add_argument("--update", action="store_true", help="write the pin (needs --all)")
    args = parser.parse_args(argv)
    if args.update and not args.all:
        parser.error("--update needs --all")
    if args.update and (args.configuration or args.route_wide):
        parser.error("--update pins all three configurations without --route-wide")
    configurations = tuple(args.configuration or CONFIGURATIONS)
    names = public_benchmarks() if args.all else args.benchmarks
    if not names:
        parser.error("name a benchmark or pass --all")

    counts: dict[str, dict[str, dict[str, int]]] = {}
    failed = False
    tasks = [
        (name, configuration, args.route_wide) for name in names for configuration in configurations
    ]
    with contextlib.ExitStack() as stack:
        if args.jobs > 1:
            # one process per run: a run shares nothing with the one before it
            pool = stack.enter_context(
                ProcessPoolExecutor(
                    args.jobs, mp_context=get_context("spawn"), max_tasks_per_child=1
                )
            )
            results = pool.map(_run, tasks)
        else:
            results = map(_run, tasks)
        for name, configuration, findings in results:
            by_check = counts.setdefault(name, {})[configuration] = count_by_check(findings)
            errors = sum(finding.severity == ERROR for finding in findings)
            failed = failed or errors > 0
            summary = ", ".join(f"{check} {count}" for check, count in by_check.items())
            print(
                f"{name} [{configuration}]: {errors} error(s); {summary or 'no findings'}",
                flush=True,
            )
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
