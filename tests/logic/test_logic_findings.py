"""The logic harness on every public benchmark in every router configuration, against the pin.

`tests/baselines/logic_findings.json` holds, per benchmark and per router
configuration (`baseline`, `contribution1`, `contribution2`), how many
findings each check of `photonic_router.logic_checks` has, so that a change
in what a stage hands on is seen. The only errors a run may have are
those of `KNOWN_ERRORS`: found by the harness, not fixed yet. Re-pin with
`scripts/logic_check.py --all --update` and give the reason in the commit.
"""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
from pathlib import Path

import pytest
from photonic_router.logic_checks import ERROR, count_by_check

REPO_ROOT = Path(__file__).resolve().parents[2]
PIN = json.loads(
    (REPO_ROOT / "tests" / "baselines" / "logic_findings.json").read_text(encoding="utf-8")
)

#: errors the harness found that are still there, per (benchmark, configuration, check)
KNOWN_ERRORS: dict[tuple[str, str, str], str] = {
    ("heater_single", "baseline", "routing.net_not_routed"): (
        "the optical router finds no path for gc_1,o1 -> mmi_0,o1; a known limit of "
        "this placement, written down in benchmarks/heater_single.py"
    ),
    ("heater_single", "contribution1", "routing.net_not_routed"): (
        "as in the baseline: no path for gc_1,o1 -> mmi_0,o1"
    ),
    ("heater_single", "contribution2", "routing.net_not_routed"): (
        "the design is not band-layered, so the standard router routes it: as in the baseline"
    ),
    ("permutation_synthetic", "baseline", "routing.net_not_routed"): (
        "the standard router does not converge on the random permutation within 10 "
        "rounds; the design is made for the two contributions, which route it"
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


RUNS = [
    (benchmark, configuration)
    for benchmark in sorted(PIN)
    for configuration in logic_check.CONFIGURATIONS
]


@pytest.mark.e2e
def test_the_pin_lists_every_public_benchmark_in_the_three_configurations() -> None:
    assert sorted(PIN) == logic_check.public_benchmarks()
    assert all(tuple(sorted(PIN[name])) == logic_check.CONFIGURATIONS for name in PIN)


def test_every_known_error_is_an_error_of_the_pin() -> None:
    """A known error that is fixed leaves the list."""
    for (benchmark, configuration, check), reason in KNOWN_ERRORS.items():
        assert PIN[benchmark][configuration].get(check, 0) > 0, reason


@pytest.mark.e2e
@pytest.mark.parametrize(("benchmark", "configuration"), RUNS)
def test_a_run_has_the_pinned_findings_and_only_known_errors(
    benchmark: str, configuration: str
) -> None:
    pinned = PIN[benchmark][configuration]
    if _is_large(benchmark) and os.environ.get("LOGIC_CHECK_LARGE") != "1":
        # The session stages take over a minute each here; the pin is still
        # written from a full run (`scripts/logic_check.py --all --update`).
        findings = logic_check.findings_for_benchmark(
            benchmark, configuration=configuration, session=False
        )
        loaded = {
            check: count
            for check, count in pinned.items()
            if check.startswith(("reading.", "crossing_plan."))
        }
        assert count_by_check(findings) == loaded
        return
    findings = logic_check.findings_for_benchmark(benchmark, configuration=configuration)
    unknown = [
        str(finding)
        for finding in findings
        if finding.severity == ERROR
        and (benchmark, configuration, finding.check) not in KNOWN_ERRORS
    ]
    assert unknown == []
    assert count_by_check(findings) == pinned


def test_the_paper_cells_are_the_27_of_the_results_table() -> None:
    cells = logic_check.PAPER_CELLS
    assert len(cells) == len(set(cells)) == 27
    assert {name for name, _ in cells} <= set(PIN)
    wide = {configuration for name, configuration in cells if "128x128" in name}
    assert wide == {"contribution2"}


def test_the_plm_cells_are_the_plm_benchmarks_on_the_standard_router() -> None:
    listed = subprocess.run(
        [str(REPO_ROOT / "scripts" / "results" / "plm_baseline.sh"), "--list"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()
    assert [name for name, _ in logic_check.PLM_CELLS] == listed
    assert {configuration for _, configuration in logic_check.PLM_CELLS} == {"baseline"}


def test_no_required_cell_has_a_known_error_or_is_refused() -> None:
    """What the router has to route, it routes: the paper's cells and the PLM benchmarks."""
    ended = {"routing.net_not_routed", "flow.standard_router_fallback", "flow.run_failed"}
    assert len(logic_check.REQUIRED_CELLS) == len(set(logic_check.REQUIRED_CELLS)) == 40
    for benchmark, configuration in logic_check.REQUIRED_CELLS:
        assert not ended & set(PIN[benchmark][configuration]), (benchmark, configuration)
        known = [key for key in KNOWN_ERRORS if key[:2] == (benchmark, configuration)]
        assert known == [], known


def test_contribution_2_falls_back_to_the_standard_router_exactly_where_bands_are_missing() -> None:
    """The fallback run has the findings of the baseline run and the note, nothing else."""
    note = "flow.standard_router_fallback"
    for benchmark, by_configuration in PIN.items():
        pinned = dict(by_configuration["contribution2"])
        outside = by_configuration["baseline"].get("crossing_plan.net_outside_band_model", 0)
        assert pinned.pop(note, 0) == (1 if outside else 0), benchmark
        if outside and not _is_wide(benchmark):
            assert pinned == by_configuration["baseline"], benchmark
    assert set(logic_check.PLM_BENCHMARKS) <= {
        name for name in PIN if note in PIN[name]["contribution2"]
    }


def _is_wide(benchmark: str) -> bool:
    return logic_check.last_stage(benchmark) == "handoff"


def test_the_wide_designs_stop_at_the_hand_off_and_the_others_run_to_the_polygons() -> None:
    assert logic_check.last_stage("benes_8x8_flat") == "realized"
    assert logic_check.last_stage("cascade_16_folded") == "realized"
    assert logic_check.last_stage("heater_single") == "realized"
    assert logic_check.last_stage("benes_16x16") == "handoff"
    assert logic_check.last_stage("multiportmmi_64x64_bands1to2") == "handoff"
    assert logic_check.last_stage("benes_16x16", route_wide=True) == "realized"
