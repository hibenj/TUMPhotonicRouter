"""`docs/CODING_STANDARDS.md` says what a tool checks; these tests check that it does.

Every rule of sections 3 to 7 ends in a level (Tool, Test or Review) and a
state (in force or proposed). A rule that claims "Tool, in force" has to
name a check the harness really runs, a proposed Tool rule has to be held by
the quality ratchet, and the table of section 8 has to list exactly the
measures of the pin. Review rules are not checkable here.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

import tomllib

REPO_ROOT = Path(__file__).resolve().parents[1]
STANDARDS = (REPO_ROOT / "docs" / "CODING_STANDARDS.md").read_text(encoding="utf-8")
PYPROJECT = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
CARGO = tomllib.loads((REPO_ROOT / "Cargo.toml").read_text(encoding="utf-8"))
PIN = json.loads(
    (REPO_ROOT / "tests" / "baselines" / "quality_ratchet.json").read_text(encoding="utf-8")
)
HARNESS_SCRIPTS = "\n".join(
    (REPO_ROOT / "scripts" / name).read_text(encoding="utf-8") for name in ("lint.sh", "check.sh")
)

_RUFF_CODE = re.compile(r"^[A-Z]+\d+$")
_CLIPPY_LINT = re.compile(r"^clippy::(\w+)$")


@dataclass(frozen=True)
class Claim:
    """One level statement of a rule, with the text that leads up to it."""

    rule: str
    level: str
    in_force: bool
    proposed: bool
    tokens: tuple[str, ...]


def _rules() -> dict[str, str]:
    """The numbered rules of sections 3 to 7, each as one line of text."""
    rules: dict[str, str] = {}
    section = 0
    current = ""
    for line in STANDARDS.splitlines():
        heading = re.match(r"^## (\d+)\. ", line)
        if heading:
            section = int(heading.group(1))
            current = ""
            continue
        if not 3 <= section <= 7:
            continue
        item = re.match(r"^(\d+)\. (.*)$", line)
        if item:
            current = f"{section}.{item.group(1)}"
            rules[current] = item.group(2)
        elif current and line.startswith(" "):
            rules[current] += " " + line.strip()
        elif line.strip():
            current = ""
    return rules


def _claims() -> list[Claim]:
    claims: list[Claim] = []
    for rule, text in _rules().items():
        plain = re.sub(r"\*\*([^*]+)\*\*", r"\1", text)
        start = 0
        for marker in re.finditer(r"\*([^*]+)\*", plain):
            words = marker.group(1)
            level = words.split()[0].rstrip(",;")
            if level not in {"Tool", "Test", "Review"}:
                continue
            claims.append(
                Claim(
                    rule=rule,
                    level=level,
                    in_force="in force" in words,
                    proposed="proposed" in words,
                    tokens=tuple(re.findall(r"`([^`]+)`", plain[start : marker.end()])),
                )
            )
            start = marker.end()
    return claims


def _ruff_selected(code: str) -> bool:
    """Whether the lint set selects a rule; `B` selects `B006` and not `BLE001`."""
    lint = PYPROJECT["tool"]["ruff"]["lint"]
    linter = code.rstrip("0123456789")
    return code not in lint["ignore"] and any(
        code.startswith(entry) and entry.rstrip("0123456789") == linter for entry in lint["select"]
    )


def _clippy_checked(lint: str) -> bool:
    return CARGO["workspace"]["lints"]["clippy"].get(lint) in {"warn", "deny"}


def _is_run_by_the_harness(token: str) -> bool:
    """Whether a quoted token names something the harness checks today."""
    clippy = _CLIPPY_LINT.match(token)
    if clippy:
        return _clippy_checked(clippy.group(1))
    if _RUFF_CODE.match(token):
        return _ruff_selected(token)
    return token in PIN or token in HARNESS_SCRIPTS


def _table_rows() -> dict[str, list[str]]:
    """Section 8: the rule number of each row and the pin names it quotes."""
    rows: dict[str, list[str]] = {}
    for line in STANDARDS.split("## 8. ")[1].splitlines():
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) >= 2 and re.fullmatch(r"[\d.]+", cells[0]):
            rows.setdefault(cells[0], []).extend(re.findall(r"`(\w+)`", cells[1]))
    return rows


def test_every_rule_states_its_level() -> None:
    with_level = {claim.rule for claim in _claims()}
    assert sorted(set(_rules()) - with_level) == []
    assert len(_rules()) >= 30, "the parser lost the rules"


def test_a_tool_rule_in_force_names_a_check_that_runs() -> None:
    unbacked = []
    for claim in _claims():
        if claim.level != "Tool" or not claim.in_force:
            continue
        cited_lints = [t for t in claim.tokens if _RUFF_CODE.match(t) or _CLIPPY_LINT.match(t)]
        if not any(_is_run_by_the_harness(t) for t in claim.tokens) or not all(
            _is_run_by_the_harness(t) for t in cited_lints
        ):
            unbacked.append((claim.rule, claim.tokens))
    assert unbacked == []


def test_a_proposed_tool_rule_is_held_by_the_ratchet() -> None:
    rows = _table_rows()
    unheld = [
        claim.rule
        for claim in _claims()
        if claim.level == "Tool" and claim.proposed and not claim.in_force
        if not rows.get(claim.rule)
    ]
    assert unheld == []


def test_a_test_rule_in_force_names_a_test_that_exists() -> None:
    missing = [
        claim.rule
        for claim in _claims()
        if claim.level == "Test" and claim.in_force
        if not any(
            token.startswith("tests/") and (REPO_ROOT / token).exists() for token in claim.tokens
        )
    ]
    assert missing == []


def test_the_table_lists_exactly_the_pinned_measures() -> None:
    quoted = {name for names in _table_rows().values() for name in names}
    assert quoted == set(PIN)


def test_a_rule_in_the_lint_set_is_not_also_counted() -> None:
    """A count that reached zero leaves the pin when the lint set takes it."""
    both = [name for name in PIN if name.startswith("ruff_") and _ruff_selected(name[5:])]
    assert both == []


def test_the_commit_hook_runs_the_fast_stage() -> None:
    hook = REPO_ROOT / "scripts" / "hooks" / "pre-commit"
    assert "scripts/check.sh" in hook.read_text(encoding="utf-8")
    assert hook.stat().st_mode & 0o111, "the hook is not executable"
