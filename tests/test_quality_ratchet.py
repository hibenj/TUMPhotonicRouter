"""The quality ratchet of `docs/CODING_STANDARDS.md` section 8.

`scripts/quality_ratchet.py` measures how far the code is from the proposed
rules and compares the counts with `tests/baselines/quality_ratchet.json`.
These tests cover its decisions (what fails, what may be pinned) and its two
counters that are not a tool's output. The measurement itself runs in
`scripts/lint.sh` and CI, not here.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "quality_ratchet.py"


def _load_ratchet() -> ModuleType:
    spec = importlib.util.spec_from_file_location("quality_ratchet", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_equal_counts_pass() -> None:
    ratchet = _load_ratchet()

    assert ratchet.compare({"a": 3, "b": 0}, {"a": 3, "b": 0}) == ([], [])


def test_a_risen_count_is_reported_as_risen() -> None:
    ratchet = _load_ratchet()

    rose, fell = ratchet.compare({"a": 3}, {"a": 4})

    assert rose == ["a: 3 -> 4"]
    assert fell == []


def test_a_fallen_count_must_be_pinned_again() -> None:
    ratchet = _load_ratchet()

    rose, fell = ratchet.compare({"a": 3}, {"a": 2})

    assert rose == []
    assert fell == ["a: 3 -> 2"]


def test_a_measure_missing_on_one_side_fails() -> None:
    ratchet = _load_ratchet()

    rose, _fell = ratchet.compare({"a": 3}, {"a": 3, "b": 1})

    assert len(rose) == 1 and rose[0].startswith("b:")


def test_update_refuses_a_rise_that_is_not_named() -> None:
    ratchet = _load_ratchet()
    pinned = {"a": 3, "b": 5}
    measured = {"a": 4, "b": 4}

    assert ratchet.repin(pinned, measured, allow_rise=[]) == ["a: 3 -> 4"]
    assert ratchet.repin(pinned, measured, allow_rise=["a"]) == []


def test_private_imports_counts_underscore_names_but_not_the_extension(tmp_path: Path) -> None:
    ratchet = _load_ratchet()
    source = tmp_path / "module.py"
    source.write_text(
        "from __future__ import annotations\n"
        "from photonic_router import _rust\n"
        "from other import _helper, public, _second\n"
        "from third import __version__\n",
        encoding="utf-8",
    )

    assert ratchet.private_imports([source]) == 2


def test_python_exceptions_counts_only_the_ratchet_rules(tmp_path: Path) -> None:
    ratchet = _load_ratchet()
    source = tmp_path / "module.py"
    source.write_text(
        "import os  # noqa: F401\n"
        "def f(a, b, c, d, e, f, g, h):  # noqa: PLR0913, C901 (binding signature)\n"
        "    return a\n",
        encoding="utf-8",
    )

    assert ratchet.python_exceptions([source]) == 2


def test_the_pin_names_every_tool_rule_of_the_script() -> None:
    ratchet = _load_ratchet()
    pinned = json.loads(ratchet.PIN_PATH.read_text(encoding="utf-8"))

    assert {f"ruff_{code}" for code in ratchet.RUFF_RULES} <= pinned.keys()
    assert {f"rust_{lint.rsplit(':', 1)[-1]}" for lint in ratchet.RUST_LINTS} <= pinned.keys()
