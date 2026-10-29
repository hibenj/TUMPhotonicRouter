#!/bin/bash
# The PLM gate (phase 6 slice 0): route heater_s_mod and cascade_8_compact
# into a fresh temporary root (both at once, they are independent runs) and
# compare GDS length, meander length, group count, groups over tolerance,
# realized spreads, nets, rc and verifier errors against the stored baseline
# docs/plm_baseline.json (scripts/results/plm_table.py --compare). Exit 0
# only when both cells match. Like gate_short.sh for the crossing engine:
# the check for every later phase 6 slice. Run from anywhere; needs the
# built kernel. About two to three minutes on the workstation.
#   GATE_KEEP=1  keeps the temporary root and prints its path.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
root="$(mktemp -d "${TMPDIR:-/tmp}/gate_plm_XXXX")"
start=$(date +%s)
for b in heater_s_mod cascade_8_compact; do
  RESULTS_ROOT="$root" "$R/scripts/results/plm_baseline.sh" "$b" &
done
wait
"$R/.venv/bin/python" "$R/scripts/results/plm_table.py" "$root" --compare "$R/docs/plm_baseline.json" --only-present
rc=$?
echo "gate wall: $(( $(date +%s) - start )) s"
if [ "${GATE_KEEP:-0}" = 1 ]; then echo "kept: $root"; else rm -rf "$root"; fi
exit $rc
