"""Guard: the `translation` package and the root `routing_flow*` modules are gone.

Every module lives under `python/photonic_router/` since phase 8 slice A. Only
the five-line `routing_flow.py` script stays at the repository root, and nothing
imports it.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCAN_DIRS = ("python", "tests", "scripts", "benchmarks")
LEGACY_ROOTS = {"translation", "routing_flow", "benchmark_metadata"}
LEGACY_NAME = re.compile(r"^(translation|routing_flow(_\w+)?|benchmark_metadata)(\.|$)")
LEGACY_STRING = re.compile(r"""["'](translation|routing_flow_\w+)\.""")


def _sources() -> list[Path]:
    paths = []
    for name in SCAN_DIRS:
        paths.extend(p for p in (ROOT / name).rglob("*.py") if ".venv" not in p.parts)
    return sorted(paths)


def test_no_import_of_removed_modules() -> None:
    offenders = []
    for path in _sources():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            names: list[str] = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                names = [node.module]
            for name in names:
                if LEGACY_NAME.match(name):
                    offenders.append(f"{path.relative_to(ROOT)}:{node.lineno}: {name}")
    assert not offenders, "imports of removed modules:\n" + "\n".join(offenders)


def test_no_string_reference_to_removed_modules() -> None:
    offenders = []
    for path in _sources():
        if path == Path(__file__).resolve():
            continue
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if LEGACY_STRING.search(line):
                offenders.append(f"{path.relative_to(ROOT)}:{lineno}: {line.strip()}")
    assert not offenders, "string references to removed modules:\n" + "\n".join(offenders)


def test_translation_directory_is_gone() -> None:
    assert not (ROOT / "translation").exists() or not any((ROOT / "translation").glob("*.py"))
    assert not any(ROOT.glob("routing_flow_*.py"))


def test_no_benchmark_module_defines_stable_blocks() -> None:
    """Benchmark defaults live in `benchmarks/<name>.toml` (phase 8 slice C)."""
    offenders = []
    for path in sorted((ROOT / "benchmarks").glob("*.py")):
        text = path.read_text(encoding="utf-8")
        if "STABLE_ROUTING_FLAGS" in text or "STABLE_ROUTING_ENV" in text:
            offenders.append(path.name)
    assert not offenders, f"STABLE_ROUTING_* still in: {offenders}"
