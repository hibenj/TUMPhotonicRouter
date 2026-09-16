#!/bin/bash
# Full archived rerun of the frozen baseline (owner go 2026-09-16 16:50), sequential:
#   ours: ladder x {lidar-pure, contribution1, contribution2}, mesh64 x 3, benes64 + benes128 contribution2;
#   then original LiDAR at price 0 (comp_LiDAR.yml): benes 8/16/32, mesh 8/16/32/64.
# Stop between steps with touch results/STOP. Every run: results/<benchmark>/<config>/<stamp>/ (see run_and_archive.sh).
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$R" || exit 1
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
log="$R/results/rerun_all.txt"; mkdir -p "$R/results"
say() { echo "$(date -Is) $*" >> "$log"; }
ours() { # <config> <benchmark> <cap_s> <args...>
  [ -f "$R/results/STOP" ] && { say "STOP: skipped $1 $2"; return; }
  local out; out=$(timeout -s TERM -k 30 "$3" scripts/results/run_and_archive.sh "$1" "$2" "${@:4}" 2>&1 | tail -2 | tr '\n' ' ')
  say "$1 $2 | $out"
}
say "rerun start commit=$(git rev-parse --short HEAD)"
LADDER="benes_4x4 multiportmmi_8x8 benes_8x8 multiportmmi_16x16 benes_16x16 multiportmmi_32x32 benes_32x32"
for b in $LADDER; do ours lidar-pure $b 3600 --crossing-mode lidar-pure; done
for b in $LADDER; do ours contribution1 $b 3600 --crossing-mode lidar-guided; done
for b in $LADDER; do ours contribution2 $b 3600 --preplaced-crossing-grids true; done
ours lidar-pure multiportmmi_64x64 7200 --crossing-mode lidar-pure
ours contribution1 multiportmmi_64x64 7200 --crossing-mode lidar-guided
ours contribution2 multiportmmi_64x64 3600 --preplaced-crossing-grids true
ours contribution2 benes_64x64 3600 --preplaced-crossing-grids true
ours contribution2 benes_128x128 3600 --preplaced-crossing-grids true
say "ours done"
# original LiDAR, price 0
L=/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute; LP=/home/benjamin/Documents/Repositories/working/LiDAR/.venv/bin/python
lidar() { # <benchmark> <cap_s>
  [ -f "$R/results/STOP" ] && { say "STOP: skipped lidar_p0 $1"; return; }
  local out="$R/results/$1/lidar_p0/$(date +%Y%m%d-%H%M%S)"; mkdir -p "$out" "$L/result/LiDAR/p0"
  local start=$(date +%s)
  (cd "$L" && timeout -s TERM -k 30 "$2" "$LP" main/picroute.py --benchmark=benchmarks/$1/$1.yml --config=config/comp_LiDAR.yml --run.output_layout_gds_path=result/LiDAR/p0/${1}_comp_LiDAR.gds > "$out/run.log" 2>&1 < /dev/null)
  local rc=$?; local wall=$(( $(date +%s) - start ))
  local it; it=$(grep -o "DrGridRoute: Iteration: *[0-9]*" "$out/run.log" | tail -1 | grep -o "[0-9]*$")
  local agg; agg=$(grep -o -E "Net: n_[0-9]+, WL: [0-9.]+ um, Accumulated bend: [0-9.]+,Crossing: [0-9]+, insertion loss: [0-9.]+, DRV: [0-9]+" "$out/run.log" | awk -F'[:,]' '{n++; wl+=$4; x+=$8; if($12+0>0) drv++} END{printf "nets=%d WL_um=%.1f physical_crossings=%d DRV_nets=%d", n, wl, x/2, drv}')
  echo "config=lidar_p0 benchmark=$1 rc=$rc wall=${wall}s iterations=$it $agg" > "$out/run.txt"
  cp "$L/result/LiDAR/p0/${1}_comp_LiDAR.gds" "$out/" 2>/dev/null
  say "lidar_p0 $1 | $(cat "$out/run.txt")"
}
for b in benes_8x8 multiportmmi_8x8 benes_16x16 multiportmmi_16x16 benes_32x32 multiportmmi_32x32; do lidar $b 3600; done
lidar multiportmmi_64x64 14400
say "rerun done"
