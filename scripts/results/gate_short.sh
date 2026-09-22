#!/bin/bash
# The reproduction gate (ExecPlan 2026-09-22 Milestone 0): route the nine
# smallest paper cells (Benes 4x4 / 8x8, ADEPT 8x8; baseline, contribution 1,
# contribution 2) into a fresh temporary root and compare crossings, GDS length
# and verifier errors against the paper's table sources. Exit 0 only when all
# nine cells match. About two minutes. Run from anywhere; needs the built kernel.
#   GATE_KEEP=1  keeps the temporary root and prints its path.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
root="$(mktemp -d "${TMPDIR:-/tmp}/gate_short_XXXX")"
RESULTS_ROOT="$root" "$R/scripts/results/reproduce_date2027.sh" short
"$R/.venv/bin/python" "$R/scripts/results/compare_date2027.py" "$root" "$R/docs/date2027_table_sources.json" --only-present
rc=$?
if [ "${GATE_KEEP:-0}" = 1 ]; then echo "kept: $root"; else rm -rf "$root"; fi
exit $rc
