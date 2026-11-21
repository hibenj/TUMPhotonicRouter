"""The logic harness on every public benchmark, against the pin.

`tests/baselines/logic_findings.json` holds, per benchmark, how many
findings each check of `photonic_router.logic_checks` has. No benchmark has
an error; the notes are pinned so that a change in what a stage hands on is
seen. Re-pin with `scripts/logic_check.py --all --update` and give the
reason in the commit.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
from photonic_router.logic_checks import ERROR, count_by_check

REPO_ROOT = Path(__file__).resolve().parents[2]
PIN = json.loads(
    (REPO_ROOT / "tests" / "baselines" / "logic_findings.json").read_text(encoding="utf-8")
)

_spec = importlib.util.spec_from_file_location(
    "logic_check", REPO_ROOT / "scripts" / "logic_check.py"
)
assert _spec is not None and _spec.loader is not None
logic_check = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(logic_check)


@pytest.mark.e2e
def test_the_pin_lists_every_public_benchmark() -> None:
    assert sorted(PIN) == logic_check.public_benchmarks()


@pytest.mark.e2e
@pytest.mark.parametrize("benchmark", sorted(PIN))
def test_a_benchmark_has_no_error_and_the_pinned_notes(benchmark: str) -> None:
    findings = logic_check.findings_for_benchmark(benchmark)
    assert [str(finding) for finding in findings if finding.severity == ERROR] == []
    assert count_by_check(findings) == PIN[benchmark]
