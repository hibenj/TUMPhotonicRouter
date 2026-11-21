"""The logic harness on every public benchmark, against the pin.

`tests/baselines/logic_findings.json` holds, per benchmark, how many
findings each check of `photonic_router.logic_checks` has, so that a change
in what a stage hands on is seen. The only errors a benchmark may have are
those of `KNOWN_ERRORS`: found by the harness, not fixed yet. Re-pin with
`scripts/logic_check.py --all --update` and give the reason in the commit.
"""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path

import pytest
from photonic_router.logic_checks import ERROR, count_by_check

REPO_ROOT = Path(__file__).resolve().parents[2]
PIN = json.loads(
    (REPO_ROOT / "tests" / "baselines" / "logic_findings.json").read_text(encoding="utf-8")
)

#: errors the harness found that are still in the code, with what they are
KNOWN_ERRORS: dict[str, str] = {}


def _is_large(benchmark: str) -> bool:
    """The 64 x 64 and 128 x 128 designs: read fully only with `LOGIC_CHECK_LARGE=1`."""
    return "64x64" in benchmark or "128x128" in benchmark


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
def test_a_benchmark_has_the_pinned_findings_and_only_known_errors(benchmark: str) -> None:
    if _is_large(benchmark) and os.environ.get("LOGIC_CHECK_LARGE") != "1":
        # The session stages take over a minute each here; the pin is still
        # written from a full run (`scripts/logic_check.py --all --update`).
        findings = logic_check.findings_for_benchmark(benchmark, session=False)
        reading = {k: v for k, v in PIN[benchmark].items() if k.startswith("reading.")}
        assert count_by_check(findings) == reading
        return
    findings = logic_check.findings_for_benchmark(benchmark)
    unknown = [
        str(finding)
        for finding in findings
        if finding.severity == ERROR and finding.check not in KNOWN_ERRORS
    ]
    assert unknown == []
    assert count_by_check(findings) == PIN[benchmark]
