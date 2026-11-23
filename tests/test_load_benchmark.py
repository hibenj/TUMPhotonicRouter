"""`flow.load_benchmark` says what is wrong with a benchmark it cannot load."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
from photonic_router import flow


@pytest.fixture
def benchmarks_package(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """An empty `benchmarks` package of this test's own, first on the import path."""
    package = tmp_path / "benchmarks"
    package.mkdir()
    (package / "__init__.py").write_text("", encoding="utf-8")
    monkeypatch.syspath_prepend(str(tmp_path))
    for name in [name for name in sys.modules if name.split(".")[0] == "benchmarks"]:
        monkeypatch.delitem(sys.modules, name)
    return package


def test_a_benchmark_that_does_not_exist_is_not_found(benchmarks_package: Path) -> None:
    with pytest.raises(ModuleNotFoundError, match="Benchmark 'nowhere' not found"):
        flow.load_benchmark("nowhere")


def test_a_missing_import_inside_a_benchmark_is_reported_as_that_import(
    benchmarks_package: Path,
) -> None:
    (benchmarks_package / "broken.py").write_text("import a_module_nobody_has\n", encoding="utf-8")

    with pytest.raises(ModuleNotFoundError, match="a_module_nobody_has") as error:
        flow.load_benchmark("broken")
    assert "not found in benchmarks" not in str(error.value)


def test_a_benchmark_without_build_schematic_says_so(benchmarks_package: Path) -> None:
    (benchmarks_package / "empty.py").write_text("X = 1\n", encoding="utf-8")

    with pytest.raises(AttributeError, match="must have a 'build_schematic\\(\\)' function"):
        flow.load_benchmark("empty")


def test_an_attribute_error_inside_build_schematic_is_not_rewritten(
    benchmarks_package: Path,
) -> None:
    (benchmarks_package / "faulty.py").write_text(
        "def build_schematic():\n    return None.no_such_attribute\n", encoding="utf-8"
    )

    with pytest.raises(AttributeError, match="no_such_attribute"):
        flow.load_benchmark("faulty")
