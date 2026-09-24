"""`docs/CONFIGURATION.md` is the generated configuration reference.

Milestone 7 of `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`:
the document is generated from the configuration dataclasses by
`scripts/generate_config_reference.py`, so a new field, a changed default, a
renamed overlay variable or a renamed command-line flag makes this test fail
until the document is regenerated:

    .venv/bin/python scripts/generate_config_reference.py
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "generate_config_reference.py"


def _load_generator():
    spec = importlib.util.spec_from_file_location("generate_config_reference", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_checked_in_configuration_reference_equals_the_generated_one():
    generator = _load_generator()
    checked_in = generator.OUTPUT_PATH.read_text(encoding="utf-8")

    assert checked_in == generator.render(), (
        f"{generator.OUTPUT_PATH.relative_to(REPO_ROOT)} is out of date; regenerate it with "
        "`.venv/bin/python scripts/generate_config_reference.py`"
    )


def test_generating_twice_gives_the_same_document():
    generator = _load_generator()

    assert generator.render() == generator.render()


def test_every_routing_config_field_is_listed_with_its_overlay_variable():
    generator = _load_generator()
    document = generator.OUTPUT_PATH.read_text(encoding="utf-8")

    # Two spot checks that the three columns come from the three sources: a
    # Rust-boundary field with its overlay name, and a flow option with its flag.
    assert "| `budget_first` | `int` | `2000000` | `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST` |" in (
        document
    )
    assert "`--preplaced-crossing-grids`" in document
    # Every name of the overlay table reaches the document, table entries and
    # specially handled names alike.
    names = {entry.name for entry in generator.env_overlay_module.ENV_OVERLAY}
    names |= set(generator.env_overlay_module.SPECIALLY_HANDLED_NAMES)
    missing = sorted(name for name in names if f"`{name}`" not in document)
    assert missing == []
