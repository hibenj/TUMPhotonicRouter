"""The check of the private circuits: what counts as routed, and what it finds."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_check() -> ModuleType:
    spec = importlib.util.spec_from_file_location(
        "private_circuits_check", REPO_ROOT / "scripts" / "private_circuits_check.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_SUMMARY = (
    "SUMMARY benchmark=private.x nets=63 failed_attempts=0 detours=0 verifier_errors={errors} "
    "verifier_warnings=0 plm_groups=17 plm_over_tolerance=0 total_s=5.1"
)


def test_a_run_passes_with_exit_code_zero_and_a_clean_verifier() -> None:
    check = _load_check()

    passed, why = check.verdict(0, "routing...\n" + _SUMMARY.format(errors=0) + "\n")

    assert passed
    assert why == "nets=63, plm_groups=17, plm_over_tolerance=0, total_s=5.1"


def test_a_run_that_ends_with_an_error_fails_with_its_last_line() -> None:
    check = _load_check()
    output = _SUMMARY.format(errors="-") + "\nRuntimeError: No route found for a_to_b\n"

    assert check.verdict(1, output) == (
        False,
        "exit code 1: RuntimeError: No route found for a_to_b",
    )


def test_a_run_without_a_clean_verifier_result_fails() -> None:
    check = _load_check()

    assert not check.verdict(0, _SUMMARY.format(errors=2))[0]
    assert not check.verdict(0, _SUMMARY.format(errors="-"))[0]
    assert check.verdict(0, "no summary at all") == (
        False,
        "no clean verifier result: no SUMMARY line",
    )


def test_the_circuits_are_the_modules_that_build_a_schematic(tmp_path: Path) -> None:
    check = _load_check()
    (tmp_path / "b_circuit.py").write_text("def build_schematic():\n    pass\n", encoding="utf-8")
    (tmp_path / "a_circuit.py").write_text("def build_schematic():\n    pass\n", encoding="utf-8")
    (tmp_path / "__init__.py").write_text("", encoding="utf-8")
    (tmp_path / "helpers.py").write_text("X = 1\n", encoding="utf-8")

    assert check.private_circuits(tmp_path) == ["a_circuit", "b_circuit"]
    assert check.private_circuits(tmp_path / "missing") == []
