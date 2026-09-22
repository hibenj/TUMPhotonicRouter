#!/bin/bash
# Reproduce every row of ours in the DATE 2027 results table, sequentially,
# on this branch's engine (frozen kernel 8ddde83 + the contribution 2
# long-straight exemption, which routing_flow.py enables whenever pre-placed
# crossing grids are on). Same benchmarks, arguments and caps as the archived
# runs (see the provenance table in docs/DATE2027_REPRODUCTION.md).
#
# Archives land in RESULTS_ROOT (default results_date2027/), never in results/,
# so the paper's archives stay untouched. Afterwards:
#   .venv/bin/python scripts/results/compare_date2027.py results_date2027 <EXPERIMENTS_TABLE_SOURCES.json>
# compares crossings, GDS lengths and verifier errors cell by cell (times are
# reported, not compared). Stop between rows with: touch results_date2027/STOP
# Usage: reproduce_date2027.sh [all|benes|mesh|c2-mesh|ladder]   (default all)
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$R" || exit 1
export RESULTS_ROOT="${RESULTS_ROOT:-$R/results_date2027}"; mkdir -p "$RESULTS_ROOT"
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
what="${1:-all}"
log="$RESULTS_ROOT/reproduce.txt"; say() { echo "$(date -Is) $*" >> "$log"; }
ours() { # <config> <benchmark> <cap_s> <args...>
  [ -f "$RESULTS_ROOT/STOP" ] && { say "STOP: skipped $1 $2"; return; }
  local out; out=$(timeout -s TERM -k 30 "$3" scripts/results/run_and_archive.sh "$1" "$2" "${@:4}" 2>&1 | tail -2 | tr '\n' ' ')
  say "$1 $2 | $out"
}
say "DATE2027 reproduction ($what) start commit=$(git rev-parse --short HEAD) kernel=$(stat -c %y python/photonic_router/_rust.abi3.so | cut -c1-19)"
BENES_LADDER="benes_4x4_flat benes_8x8_flat benes_16x16_flat benes_32x32_flat"
MESH_LADDER="multiportmmi_8x8 multiportmmi_16x16 multiportmmi_32x32 multiportmmi_64x64"
case "$what" in
  all|benes|ladder)
    for b in $BENES_LADDER; do ours lidar-pure $b 3600 --crossing-mode lidar-pure; done
    for b in $BENES_LADDER; do ours contribution1 $b 3600 --crossing-mode lidar-guided; done
    for b in $BENES_LADDER; do ours contribution2 $b 3600 --preplaced-crossing-grids true --verbose-routes; done
    ;;&
  all|benes)
    ours contribution2 benes_64x64_flat 3600 --preplaced-crossing-grids true --verbose-routes
    # benes_128x128_flat: routing loop 925 s, but the verifier alone took 4449 s (20480 routes)
    ours contribution2 benes_128x128_flat 10800 --preplaced-crossing-grids true --verbose-routes
    ;;&
  all|mesh)
    for b in $MESH_LADDER; do ours lidar-pure $b 3600 --crossing-mode lidar-pure; done
    for b in $MESH_LADDER; do ours contribution1 $b 3600 --crossing-mode lidar-guided; done
    ;;&
  all|mesh|c2-mesh)
    for b in $MESH_LADDER multiportmmi_128x128; do ours contribution2 $b 3600 --preplaced-crossing-grids true --verbose-routes; done
    ;;
esac
say "DATE2027 reproduction ($what) done"
