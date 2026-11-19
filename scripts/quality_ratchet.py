#!/usr/bin/env python
"""Measure the code against `docs/CODING_STANDARDS.md` and hold the counts.

Every proposed Tool rule of the coding standards has a count of the places
that do not meet it yet. This script measures those counts and compares them
with the pinned ones in `tests/baselines/quality_ratchet.json`:

* a count that rose fails the check;
* a count that fell fails it too, until the pin is rewritten with
  `--update`, so that the gain cannot be spent again later;
* `--update` refuses to pin a higher count unless the measure is named with
  `--allow-rise` (a new, reasoned exception; the commit message says why).

The limits themselves are configuration, not this script: `max-args` and
`max-statements` under `[tool.ruff.lint.pylint]` and `max-complexity` under
`[tool.ruff.lint.mccabe]` in `pyproject.toml`, `too-many-lines-threshold`
in `clippy.toml`.

Usage, from the repository root:

    .venv/bin/python scripts/quality_ratchet.py                 # check
    .venv/bin/python scripts/quality_ratchet.py --update        # re-pin falls
    .venv/bin/python scripts/quality_ratchet.py --update --allow-rise python_exceptions

`scripts/lint.sh` and CI run the check.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import subprocess
import sys
from collections import Counter
from collections.abc import Callable, Iterable, Sequence
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE = REPO_ROOT / "python" / "photonic_router"
RUST_CORE = REPO_ROOT / "crates" / "router-core"
PIN_PATH = REPO_ROOT / "tests" / "baselines" / "quality_ratchet.json"

#: Ruff rules the standards propose and the lint set does not select yet,
#: with the rule of `docs/CODING_STANDARDS.md` each one stands for.
RUFF_RULES: dict[str, str] = {
    "ANN001": "4.2 parameter without an annotation",
    "ANN201": "4.2 public function without a return annotation",
    "ANN202": "4.2 private function without a return annotation",
    "PLR0913": "4.4 function over the parameter limit",
    "PLR0915": "4.5 function over the statement limit",
    "C901": "4.5 function over the complexity limit",
    "T201": "4.6 print in the package",
    "D100": "4.7 public module without a docstring",
    "D101": "4.7 public class without a docstring",
    "D102": "4.7 public method without a docstring",
    "D103": "4.7 public function without a docstring",
    "D104": "4.7 public package without a docstring",
    "BLE001": "4.9 blind except",
}

#: Clippy and rustc lints the standards propose for `router-core`.
RUST_LINTS: dict[str, str] = {
    "clippy::unwrap_used": "5.3 unwrap() outside tests",
    "missing_docs": "5.4 public item without a doc comment",
    "clippy::too_many_lines": "5.5 function over the line limit",
}

#: Workspace allows that rule 5.2 calls debt.
RUST_DEBT_ALLOWS = ("too_many_arguments", "type_complexity")

_UNTYPED_DICT = re.compile(r"\bdict\[str, (?:object|Any)\]")
_NOQA = re.compile(r"#\s*noqa:\s*([A-Z0-9, ]+)")
_RUST_ALLOW = re.compile(r"#!?\[(?:allow|expect)\(([^)]*)\)\]")


def _python_sources(root: Path) -> list[Path]:
    """Return the Python files below `root`, sorted."""
    return sorted(path for path in root.rglob("*.py") if "__pycache__" not in path.parts)


def _run(command: Sequence[str], env: dict[str, str] | None = None) -> str:
    """Run a tool from the repository root and return what it printed.

    The tools measured here exit non-zero when they find something, so the
    exit code is not an error; a tool that cannot start raises.
    """
    completed = subprocess.run(
        command, cwd=REPO_ROOT, capture_output=True, text=True, check=False, env=env
    )
    return completed.stdout


def ruff_counts(package: Path = PACKAGE) -> dict[str, int]:
    """Count the findings of each rule of `RUFF_RULES` in the package."""
    output = _run(
        [
            sys.executable,
            "-m",
            "ruff",
            "check",
            str(package),
            "--select",
            ",".join(RUFF_RULES),
            "--output-format",
            "json",
            "--no-fix",
            "--exit-zero",
        ]
    )
    found = Counter(finding["code"] for finding in json.loads(output))
    return {f"ruff_{code}": found.get(code, 0) for code in RUFF_RULES}


def mypy_errors() -> int:
    """Count mypy's errors on the whole package (rule 4.2).

    `pyproject.toml` lists a narrower set of files for the informational
    run; the rule is about the package, so the package is what is counted.
    """
    output = _run([sys.executable, "-m", "mypy", str(PACKAGE), "routing_flow.py"])
    match = re.search(r"Found (\d+) errors?", output)
    if match:
        return int(match.group(1))
    if "Success" in output:
        return 0
    raise RuntimeError(f"mypy printed no verdict:\n{output[-2000:]}")


def private_imports(sources: Iterable[Path]) -> int:
    """Count names with a leading underscore imported from another module (rule 3.5).

    `from x import _name` counts once per name. The compiled extension
    `_rust` is a module, not a private name of one, and does not count.
    """
    count = 0
    for path in sources:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or node.module == "__future__":
                continue
            count += sum(
                1
                for alias in node.names
                if alias.name.startswith("_")
                and not alias.name.startswith("__")
                and alias.name != "_rust"
            )
    return count


def untyped_dict_annotations(sources: Iterable[Path]) -> int:
    """Count `dict[str, object]` and `dict[str, Any]` in the sources (rule 4.3)."""
    return sum(len(_UNTYPED_DICT.findall(path.read_text(encoding="utf-8"))) for path in sources)


def class_field_count(path: Path, class_name: str) -> int:
    """Count the annotated fields of one class (rule 3.2 for the session state)."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return sum(isinstance(statement, ast.AnnAssign) for statement in node.body)
    raise RuntimeError(f"{path}: no class {class_name}")


def python_exceptions(sources: Iterable[Path]) -> int:
    """Count `# noqa` exceptions to the rules of `RUFF_RULES` (section 2)."""
    count = 0
    for path in sources:
        for codes in _NOQA.findall(path.read_text(encoding="utf-8")):
            count += sum(code.strip() in RUFF_RULES for code in codes.split(","))
    return count


def rust_lint_counts() -> dict[str, int]:
    """Count the findings of each lint of `RUST_LINTS` in `router-core`'s library.

    Only the library target is checked, so test code is not counted.
    """
    command = ["cargo", "clippy", "-q", "-p", "router-core", "--lib", "--message-format", "json"]
    command += ["--", "-A", "clippy::all"]
    for lint in RUST_LINTS:
        command += ["-W", lint]
    output = _run(command, env={**os.environ, "CLIPPY_CONF_DIR": str(REPO_ROOT)})
    found: Counter[str] = Counter()
    finished = False
    for line in output.splitlines():
        if not line.startswith("{"):
            continue
        message = json.loads(line)
        if message.get("reason") == "build-finished":
            finished = bool(message.get("success"))
        code = (message.get("message") or {}).get("code") or {}
        if message.get("reason") == "compiler-message" and code.get("code") in RUST_LINTS:
            found[code["code"]] += 1
    if not finished:
        raise RuntimeError("cargo clippy did not finish; run scripts/lint.sh for its output")
    return {f"rust_{lint.rsplit(':', 1)[-1]}": found.get(lint, 0) for lint in RUST_LINTS}


def rust_exceptions(source_root: Path = RUST_CORE / "src") -> int:
    """Count the exceptions in `router-core` that name a ratchet lint.

    Both spellings count: `#[expect(lint, reason = "...")]`, the one section
    2 asks for, and the older `#[allow(lint)]`.
    """
    watched = {lint.rsplit(":", 1)[-1] for lint in RUST_LINTS} | set(RUST_DEBT_ALLOWS)
    count = 0
    for path in sorted(source_root.rglob("*.rs")):
        for lints in _RUST_ALLOW.findall(path.read_text(encoding="utf-8")):
            count += sum(lint.strip().rsplit(":", 1)[-1] in watched for lint in lints.split(","))
    return count


def workspace_debt_allows(cargo_toml: Path = REPO_ROOT / "Cargo.toml") -> int:
    """Count the workspace-level clippy allows that rule 5.2 calls debt."""
    text = cargo_toml.read_text(encoding="utf-8")
    return sum(
        bool(re.search(rf'^{name}\s*=\s*"allow"', text, flags=re.MULTILINE))
        for name in RUST_DEBT_ALLOWS
    )


def measure() -> dict[str, int]:
    """Measure every count of the ratchet on the working tree."""
    sources = _python_sources(PACKAGE)
    counts = ruff_counts()
    counts["mypy_errors"] = mypy_errors()
    counts["private_imports"] = private_imports(sources)
    counts["untyped_dict_annotations"] = untyped_dict_annotations(sources)
    counts["session_state_fields"] = class_field_count(
        PACKAGE / "routing" / "state.py", "SessionState"
    )
    counts["python_exceptions"] = python_exceptions(sources)
    counts.update(rust_lint_counts())
    counts["rust_exceptions"] = rust_exceptions()
    counts["rust_workspace_debt_allows"] = workspace_debt_allows()
    return dict(sorted(counts.items()))


def compare(pinned: dict[str, int], measured: dict[str, int]) -> tuple[list[str], list[str]]:
    """Return the measures that rose and the measures that fell, as report lines.

    A measure that is missing on one side counts as risen: the pin and the
    script must name the same measures.
    """
    rose: list[str] = []
    fell: list[str] = []
    for name in sorted(pinned.keys() | measured.keys()):
        if name not in pinned or name not in measured:
            rose.append(f"{name}: measured and pinned measures differ; re-pin with --update")
        elif measured[name] > pinned[name]:
            rose.append(f"{name}: {pinned[name]} -> {measured[name]}")
        elif measured[name] < pinned[name]:
            fell.append(f"{name}: {pinned[name]} -> {measured[name]}")
    return rose, fell


def repin(pinned: dict[str, int], measured: dict[str, int], allow_rise: Sequence[str]) -> list[str]:
    """Return the measures that would rise without being named in `allow_rise`."""
    return [
        f"{name}: {pinned[name]} -> {count}"
        for name, count in measured.items()
        if name in pinned and count > pinned[name] and name not in allow_rise
    ]


def _report(title: str, lines: Sequence[str], out: Callable[[str], None]) -> None:
    """Print a titled block of report lines."""
    out(title)
    for line in lines:
        out(f"  {line}")


def main(argv: Sequence[str] | None = None) -> int:
    """Check the working tree against the pin, or rewrite the pin."""
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--update", action="store_true", help="rewrite the pin from this run")
    parser.add_argument(
        "--allow-rise",
        action="append",
        default=[],
        metavar="MEASURE",
        help="with --update: accept a higher count for this measure",
    )
    arguments = parser.parse_args(argv)
    measured = measure()
    pinned: dict[str, int] = (
        json.loads(PIN_PATH.read_text(encoding="utf-8")) if PIN_PATH.exists() else {}
    )
    if arguments.update:
        refused = repin(pinned, measured, arguments.allow_rise)
        if refused:
            _report("quality ratchet: not pinned, these counts would rise:", refused, print)
            return 1
        PIN_PATH.write_text(json.dumps(measured, indent=2) + "\n", encoding="utf-8")
        print(f"quality ratchet: pinned {len(measured)} measures in {PIN_PATH.name}")
        return 0
    rose, fell = compare(pinned, measured)
    if rose:
        _report("quality ratchet: these counts rose (docs/CODING_STANDARDS.md):", rose, print)
    if fell:
        _report("quality ratchet: these counts fell; re-pin with --update:", fell, print)
    if rose or fell:
        return 1
    print(f"quality ratchet: {len(measured)} measures match the pin")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
