#!/bin/bash
# benes_<n>x<n>_flat on the frozen baseline engine (commit 8ddde83, the engine
# of every archived lidar-pure / contribution 1 row), owner rule 2026-09-16
# (frozen baseline) + 2026-09-19 (identical Benes netlists). The frozen engine
# lives in the worktree TUMPhotonicRouter-frozen (kernel built there from
# 8ddde83; flat benchmark files copied in); PYTHONPATH puts its python/ before
# the venv's .pth entry, so kernel and photonic_router package are the frozen
# ones. Archives land in the main results/ (symlink). Stop with results/STOP.
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)-frozen"
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$W" || exit 1
export PYTHONPATH="$W/python"
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
log="$R/results/rerun_benes_flat_frozen.txt"
say() { echo "$(date -Is) $*" >> "$log"; }
ours() { # <config> <benchmark> <cap_s> <args...>
  [ -f "$R/results/STOP" ] && { say "STOP: skipped $1 $2"; return; }
  local out; out=$(timeout -s TERM -k 30 "$3" "$W/scripts/results/run_and_archive.sh" "$1" "$2" "${@:4}" 2>&1 | tail -2 | tr '\n' ' ')
  say "$1 $2 | $out"
}
say "benes flat rerun (frozen engine) start commit=$(git -C "$W" rev-parse --short HEAD) kernel=$(stat -c %y "$W/python/photonic_router/_rust.abi3.so" | cut -c1-19)"
LADDER="benes_4x4_flat benes_8x8_flat benes_16x16_flat benes_32x32_flat"
for b in $LADDER; do ours lidar-pure $b 3600 --crossing-mode lidar-pure; done
for b in $LADDER; do ours contribution1 $b 3600 --crossing-mode lidar-guided; done
for b in $LADDER; do ours contribution2 $b 3600 --preplaced-crossing-grids true --verbose-routes; done
ours contribution2 benes_64x64_flat 3600 --preplaced-crossing-grids true --verbose-routes
ours contribution2 benes_128x128_flat 3600 --preplaced-crossing-grids true --verbose-routes
say "benes flat rerun (frozen engine) done"
