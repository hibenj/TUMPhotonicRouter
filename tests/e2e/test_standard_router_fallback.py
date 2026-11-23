"""Contribution 2 on a design that is not band-layered: the standard router routes it.

Pre-placed crossing grids need every net confined to a band. A design
without that (the path-length-matching benchmarks) is routed as
`--configuration baseline` routes it, and the run warns once (owner,
2026-11-22).
"""

from __future__ import annotations

import logging
import re

import pytest
from photonic_router import cli, flow


def _summary(output: str) -> str:
    """The run's SUMMARY line without its run time."""
    line = next(line for line in output.splitlines() if line.startswith("SUMMARY "))
    return re.sub(r" total_s=\S+", "", line)


def _warnings(caplog: pytest.LogCaptureFixture) -> list[str]:
    """The warnings of the run; its reports are log records too and are left out."""
    return [r.getMessage() for r in caplog.records if r.levelno >= logging.WARNING]


@pytest.mark.e2e
def test_contribution_2_routes_a_plm_benchmark_as_the_standard_router_does(
    capsys: pytest.CaptureFixture[str], caplog: pytest.LogCaptureFixture
) -> None:
    cli.main(["cascade_8_compact", "--configuration", "baseline"])
    baseline = _summary(capsys.readouterr().out)
    assert _warnings(caplog) == []

    cli.main(["cascade_8_compact", "--configuration", "contribution2"])
    output = capsys.readouterr().out
    assert _summary(output) == baseline
    assert "Placing placement-derived crossing grids" not in output
    [warning] = _warnings(caplog)
    assert warning.startswith(flow.STANDARD_ROUTER_FALLBACK)
    assert "25 net(s) outside the band model (non_adjacent_walls: 25)" in warning


@pytest.mark.e2e
def test_contribution_2_on_a_band_layered_design_does_not_fall_back(
    capsys: pytest.CaptureFixture[str], caplog: pytest.LogCaptureFixture
) -> None:
    cli.main(["benes_4x4_flat", "--configuration", "contribution2"])
    assert "Placing placement-derived crossing grids" in capsys.readouterr().out
    assert _warnings(caplog) == []
