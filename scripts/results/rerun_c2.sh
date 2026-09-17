#!/bin/bash
# Contribution 2 rerun of the mesh rows with the dense-fan-out penalty exemption (owner go 2026-09-17 16:05; Benes rows have no dense instances and are unaffected), --verbose-routes for lengths.
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$R" || exit 1
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
log="$R/results/rerun_all.txt"; say() { echo "$(date -Is) $*" >> "$log"; }
say "contribution 2 rerun (dense fan-out exemption) start commit=$(git rev-parse --short HEAD)"
for b in multiportmmi_8x8 multiportmmi_16x16 multiportmmi_32x32 multiportmmi_64x64 multiportmmi_128x128; do
  [ -f "$R/results/STOP" ] && { say "STOP: skipped contribution2 $b"; continue; }
  out=$(timeout -s TERM -k 30 3600 scripts/results/run_and_archive.sh contribution2 $b --preplaced-crossing-grids true --verbose-routes 2>&1 | tail -2 | tr '\n' ' '); say "contribution2 $b | $out"
done
say "contribution 2 rerun done"
