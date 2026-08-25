#!/usr/bin/env bash
# Run every lint/format check this repository uses, the same way locally
# and in CI. See .agent/execplans/2026-08-25-python-rust-linting-and-coding-standards.md
# for why this is a plain script rather than a task-runner framework like
# `nox` -- this repository does not use one anywhere else, and every other
# validation command in this repository (see .agent/*.md ExecPlans) is
# already invoked as a plain, direct command.
#
# Usage: scripts/lint.sh [--fix]
#   --fix   Apply ruff's and clippy's safe auto-fixes instead of only
#           checking. Still requires re-running the project's test/
#           benchmark ladder afterward -- see the ExecPlan above; this
#           script does not do that for you.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

FIX=0
if [[ "${1:-}" == "--fix" ]]; then
    FIX=1
fi

echo "== ruff format =="
if [[ "$FIX" == 1 ]]; then
    .venv/bin/python -m ruff format .
else
    .venv/bin/python -m ruff format --check .
fi

echo "== ruff check =="
if [[ "$FIX" == 1 ]]; then
    .venv/bin/python -m ruff check --fix .
else
    .venv/bin/python -m ruff check .
fi

echo "== mypy =="
.venv/bin/python -m mypy

echo "== cargo fmt =="
if [[ "$FIX" == 1 ]]; then
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu cargo fmt
else
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu cargo fmt -- --check
fi

echo "== cargo clippy =="
if [[ "$FIX" == 1 ]]; then
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" \
        cargo clippy --fix --lib -p photonic-router --allow-dirty
else
    RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python" \
        cargo clippy --lib -- -D warnings
fi

echo "All checks passed."
