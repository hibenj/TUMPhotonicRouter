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

#: errors the harness found that are still there, per (benchmark, check), with what they are
KNOWN_ERRORS: dict[tuple[str, str], str] = {
    ("heater_single", "routing.net_not_routed"): (
        "the optical router finds no path for gc_1,o1 -> mmi_0,o1; a known limit of "
        "this placement, written down in benchmarks/heater_single.py"
    ),
    ("permutation_synthetic", "routing.net_not_routed"): (
        "the default run (baseline, no pre-placed crossings) does not converge on the "
        "random permutation; the design is made for contribution 2"
    ),
}


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
        loaded = {
            check: count
            for check, count in PIN[benchmark].items()
            if check.startswith(("reading.", "crossing_plan."))
        }
        assert count_by_check(findings) == loaded
        return
    findings = logic_check.findings_for_benchmark(benchmark)
    unknown = [
        str(finding)
        for finding in findings
        if finding.severity == ERROR and (benchmark, finding.check) not in KNOWN_ERRORS
    ]
    assert unknown == []
    assert count_by_check(findings) == PIN[benchmark]


def test_the_wide_designs_stop_at_the_hand_off_and_the_others_run_to_the_polygons() -> None:
    assert logic_check.last_stage("benes_8x8_flat") == "realized"
    assert logic_check.last_stage("cascade_16_folded") == "realized"
    assert logic_check.last_stage("heater_single") == "realized"
    assert logic_check.last_stage("benes_16x16") == "handoff"
    assert logic_check.last_stage("multiportmmi_64x64_bands1to2") == "handoff"
