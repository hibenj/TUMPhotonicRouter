#!/bin/bash
# Benes with expanded switches (benes_<n>x<n>_flat: the netlist LiDAR routes),
# owner go 2026-09-19 13:05, sequential: ladder x {lidar-pure, contribution1,
# contribution2}, then contribution 2 on 64x64 and 128x128. Same caps as
# rerun_all.sh. Stop between steps with touch results/STOP.
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$R" || exit 1
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
log="$R/results/rerun_benes_flat.txt"; mkdir -p "$R/results"
say() { echo "$(date -Is) $*" >> "$log"; }
ours() { # <config> <benchmark> <cap_s> <args...>
  [ -f "$R/results/STOP" ] && { say "STOP: skipped $1 $2"; return; }
  local out; out=$(timeout -s TERM -k 30 "$3" scripts/results/run_and_archive.sh "$1" "$2" "${@:4}" 2>&1 | tail -2 | tr '\n' ' ')
  say "$1 $2 | $out"
}
say "benes flat rerun start commit=$(git rev-parse --short HEAD)"
LADDER="benes_4x4_flat benes_8x8_flat benes_16x16_flat benes_32x32_flat"
for b in $LADDER; do ours lidar-pure $b 3600 --crossing-mode lidar-pure; done
for b in $LADDER; do ours contribution1 $b 3600 --crossing-mode lidar-guided; done
# contribution 2 writes no crossing report; the lengths come from the --verbose-routes lines (as in rerun_c2.sh)
for b in $LADDER; do ours contribution2 $b 3600 --preplaced-crossing-grids true --verbose-routes; done
ours contribution2 benes_64x64_flat 3600 --preplaced-crossing-grids true --verbose-routes
ours contribution2 benes_128x128_flat 3600 --preplaced-crossing-grids true --verbose-routes
say "benes flat rerun done"
