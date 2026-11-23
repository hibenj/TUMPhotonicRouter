"""`reporting.show_reports`: the console shows the package's reports as `print` did."""

from __future__ import annotations

import logging

import pytest
from photonic_router import reporting


def test_reports_are_lines_on_stdout_and_warnings_lines_on_stderr(
    capsys: pytest.CaptureFixture[str],
) -> None:
    reporting.show_reports()
    logger = logging.getLogger("photonic_router.some_stage")

    logger.info("      - grid: 3x4")
    logger.info("")
    logger.warning("fallback to the standard router")
    logger.debug("not shown")

    captured = capsys.readouterr()
    assert captured.out == "      - grid: 3x4\n\n"
    assert captured.err == "fallback to the standard router\n"


def test_showing_the_reports_twice_does_not_show_a_report_twice(
    capsys: pytest.CaptureFixture[str],
) -> None:
    reporting.show_reports()
    reporting.show_reports()

    logging.getLogger("photonic_router.flow").info("once")

    assert capsys.readouterr().out == "once\n"
