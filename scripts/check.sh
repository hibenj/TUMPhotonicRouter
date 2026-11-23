#!/usr/bin/env bash
# The one entry to every check of this repository
# (docs/CODING_STANDARDS.md section 2).
#
# Usage: scripts/check.sh [--full]
#   (no flag)  The fast stage, about ten seconds: scripts/lint.sh (format,
#              lint sets, generated files, quality ratchet, clippy) and the
#              tests of the harness itself. The commit hook
#              (scripts/hooks/pre-commit) runs this stage.
#   --full     The fast stage, then the kernel is rebuilt and the behaviour
#              is checked: scripts/test_baseline.sh, the two gates and, where
#              benchmarks/private/ exists, the private circuits.
#              About fifteen minutes; run it before a slice is committed.
#
# The script stops at the first stage that fails and exits with its code.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

FULL=0
case "${1:-}" in
    "") ;;
    --full) FULL=1 ;;
    *) echo "usage: scripts/check.sh [--full]" >&2; exit 2 ;;
esac

scripts/lint.sh

echo "== harness tests =="
.venv/bin/python -m pytest -q tests/test_coding_standards.py tests/test_quality_ratchet.py

if [[ "$FULL" == 1 ]]; then
    echo "== build the kernel =="
    .venv/bin/maturin develop --release
    echo "== test baseline =="
    scripts/test_baseline.sh
    echo "== gate: paper cells =="
    scripts/results/gate_short.sh
    echo "== gate: PLM cells =="
    scripts/results/plm_gate.sh
    echo "== private circuits =="
    .venv/bin/python scripts/private_circuits_check.py
fi

echo "check: all stages passed ($([[ "$FULL" == 1 ]] && echo full || echo fast))."
