"""`experiments/table.py`: the comparison table of the experiment records."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("table", REPO_ROOT / "experiments" / "table.py")
assert _spec is not None and _spec.loader is not None
table = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(table)

BENCHMARKS = table.ORDER[:2]


def _records(path: Path, lengths: list[float | None]) -> Path:
    cells = [
        {
            "benchmark": benchmark,
            "config": "lidar-pure",
            "rc": 0 if length is not None else 1,
            "metrics": {"gds_length_um": length, "crossing_count": 0} if length else {},
            "wall_s": 1.0,
        }
        for benchmark, length in zip(BENCHMARKS, lengths, strict=True)
    ]
    path.write_text(json.dumps({"cells": cells}), encoding="utf-8")
    return path


def test_a_failed_reference_cell_is_left_out_of_the_mean_and_does_not_end_the_table(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    reference = _records(tmp_path / "reference.json", [100.0, None])
    variant = _records(tmp_path / "variant.json", [200.0, 50.0])
    monkeypatch.setattr(sys, "argv", ["table.py", str(reference), str(variant)])

    assert table.main() == 0

    lines = capsys.readouterr().out.splitlines()
    assert "| 0 of 2 |" in lines[-2]
    # one cell has a ratio, 200 / 100: the mean is over that cell, not over both
    assert "2.0000 / 1.00" in lines[-1]


def test_the_pins_take_the_current_block_where_it_has_a_value(tmp_path: Path) -> None:
    pins = tmp_path / "pins.json"
    source = {
        "ours_benchmark": "b",
        "baseline": {"gds_length_um": 10.0, "current": {"crossing_count": 3}},
    }
    pins.write_text(json.dumps({"sources": {"s": source}}), encoding="utf-8")

    assert table.load_pins(pins)[("b", "lidar-pure")] == {
        "ok": True,
        "x": 3,
        "len": 10.0,
        "err": 0,
        "rounds": None,
        "wall": None,
    }
