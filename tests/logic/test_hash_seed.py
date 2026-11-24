"""A run does not depend on the hash seed of the process.

Python orders a set of strings by a hash it seeds per process, so code
that iterates a set of net or instance names and lets the order reach a
product gives a different run from process to process. Until 2026-11-24
the column grid of contribution 2 did (`column_grid_stage`). Each cell
here is routed in two processes with different seeds; the routed records
(their order, their centerlines, their lengths) have to be the same.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]

#: one cell per way of routing: the standard router with path-length matching,
#: the guided search, the X array and the column grid of the pre-placed crossings
CELLS = (
    ("cascade_8_compact", "baseline"),
    ("benes_8x8_flat", "contribution1"),
    ("benes_8x8_flat", "contribution2"),
    ("multiportmmi_8x8", "contribution2"),
)

_FINGERPRINT_SCRIPT = """
import contextlib, hashlib, io, sys
from photonic_router import cli
from photonic_router.routing.stage_observer import observe_stages

seen = []

def observer(stage, settings, product):
    if stage == "records":
        seen.append(
            [
                (record.net_name, tuple(record.corrected_centerline_um), record.total_length_um)
                for record in product.routed_net_records
            ]
        )

with observe_stages(observer), contextlib.redirect_stdout(io.StringIO()):
    cli.main([sys.argv[1], "--configuration", sys.argv[2]])
print("RECORDS", len(seen[-1]), hashlib.sha256(repr(seen).encode()).hexdigest())
"""


def _records_fingerprint(benchmark: str, configuration: str, seed: str, cwd: Path) -> str:
    environment = dict(os.environ, PYTHONHASHSEED=seed)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(REPO_ROOT), str(REPO_ROOT / "python"), environment.get("PYTHONPATH", "")]
    )
    output = subprocess.run(
        [sys.executable, "-c", _FINGERPRINT_SCRIPT, benchmark, configuration],
        capture_output=True,
        text=True,
        check=True,
        cwd=cwd,
        env=environment,
    ).stdout
    return next(line for line in output.splitlines() if line.startswith("RECORDS "))


@pytest.mark.e2e
@pytest.mark.parametrize(("benchmark", "configuration"), CELLS)
def test_a_run_has_the_same_records_under_two_hash_seeds(
    benchmark: str, configuration: str, tmp_path: Path
) -> None:
    first = _records_fingerprint(benchmark, configuration, "1", tmp_path)
    second = _records_fingerprint(benchmark, configuration, "2", tmp_path)
    assert first.split()[1] != "0", "the run reported no records"
    assert first == second
