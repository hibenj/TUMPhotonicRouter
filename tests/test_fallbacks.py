"""The fallback register of `photonic_router.records.fallbacks` and the code agree."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
from photonic_router.records import fallbacks

PACKAGE = Path(__file__).resolve().parents[1] / "python" / "photonic_router"


def _names_in_the_package() -> tuple[set[str], set[str]]:
    """The literal names passed to `fallbacks.take`, and every string constant outside the register."""
    literal: set[str] = set()
    strings: set[str] = set()
    for path in sorted(PACKAGE.rglob("*.py")):
        if path.name == "fallbacks.py":
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                strings.add(node.value)
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "take"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "fallbacks"
                and isinstance(node.args[0], ast.Constant)
            ):
                literal.add(str(node.args[0].value))
    return literal, strings


def test_every_take_names_a_registered_fallback() -> None:
    literal, _ = _names_in_the_package()
    assert sorted(literal - set(fallbacks.FALLBACKS)) == []


def test_every_registered_fallback_is_named_where_it_is_taken() -> None:
    _, strings = _names_in_the_package()
    assert sorted(set(fallbacks.FALLBACKS) - strings) == []


def test_take_counts_and_reset_forgets(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(fallbacks.FALLBACKS, "test.once", "a fallback of this test")
    monkeypatch.setitem(fallbacks.FALLBACKS, "test.per_net", "another one")
    fallbacks.reset()
    fallbacks.take("test.once")
    fallbacks.take("test.per_net", 3)

    assert fallbacks.taken() == {"test.once": 1, "test.per_net": 3}
    fallbacks.reset()
    assert fallbacks.taken() == {}


def test_an_unnamed_fallback_cannot_be_taken() -> None:
    with pytest.raises(KeyError, match="name it in FALLBACKS first"):
        fallbacks.take("test.not_a_fallback")
