"""Run the logic harness on benchmarks: does what each stage hands on make sense?

Usage:
    logic_check.py <benchmark> [<benchmark> ...] [--list N]
    logic_check.py --all [--list N]
    logic_check.py --all --update

Runs the checks of `photonic_router.logic_checks` stage by stage and prints,
per benchmark, how many findings each check has; `--list N` also prints the
first N findings of each check. Exits 1 when a benchmark has an error.

`--all` takes every module of `benchmarks/` with a `build_schematic()`
(not `benchmarks/private/`; name those modules as `private.<name>`).
`--update` writes the counts to `tests/baselines/logic_findings.json`, the
pin that `tests/logic/test_logic_findings.py` compares against; a pin that
moves is re-pinned with the reason in the commit.
"""

from __future__ import annotations

import argparse
import importlib
import json
import pkgutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from photonic_router.flow import load_benchmark
from photonic_router.logic_checks import ERROR, Finding, count_by_check
from photonic_router.logic_checks.reading import check_layout, check_schematic
from photonic_router.records.layout_from_schematic import layout_from_schematic

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


def findings_for_benchmark(benchmark: str) -> list[Finding]:
    """Every finding of every stage implemented so far, in stage order."""
    schematic = load_benchmark(benchmark)
    findings = check_schematic(schematic)
    findings.extend(check_layout(schematic, layout_from_schematic(schematic)))
    return findings


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
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
