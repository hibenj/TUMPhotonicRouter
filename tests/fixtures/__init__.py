"""Shared test fixtures (ExecPlan Milestone 6, Slice 2).

`synthetic_layouts` holds the layout/schematic/component builders that used
to be duplicated across several test files; `sessions` holds the
`SessionSettings`/`SessionState`/pipeline builders. `tests/conftest.py`
exposes the ones tests consume as pytest fixtures; the rest are imported
directly where the parameters they need vary per test.
"""

from __future__ import annotations
