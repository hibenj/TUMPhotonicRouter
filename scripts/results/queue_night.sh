#!/bin/bash
# Night queue 2026-09-16 (owner go 22:20): wait for the running LiDAR price-0 mesh64 -> contribution 2 x 9 with
# --verbose-routes (route lengths for the archive) -> original LiDAR at price 300 (comp_LiDAR_x200.yml):
# benes 8/16/32, mesh 8/16/32/64. Archived like rerun_all.sh; stop between steps with touch results/STOP.
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$R" || exit 1
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
log="$R/results/rerun_all.txt"
say() { echo "$(date -Is) $*" >> "$log"; }
say "night queue start commit=$(git rev-parse --short HEAD): waiting for the LiDAR p0 mesh64 run"
while pgrep -f "main/picroute.p[y]" > /dev/null; do sleep 60; done
say "LiDAR p0 done, contribution 2 with verbose routes"
ours() { [ -f "$R/results/STOP" ] && { say "STOP: skipped $1 $2"; return; }
  local out; out=$(timeout -s TERM -k 30 "$3" scripts/results/run_and_archive.sh "$1" "$2" "${@:4}" 2>&1 | tail -2 | tr '\n' ' '); say "$1 $2 | $out"; }
for b in benes_4x4 multiportmmi_8x8 benes_8x8 multiportmmi_16x16 benes_16x16 multiportmmi_32x32 benes_32x32 multiportmmi_64x64 benes_64x64 benes_128x128; do
  ours contribution2 $b 3600 --preplaced-crossing-grids true --verbose-routes
done
say "contribution 2 verbose done, LiDAR price 300"
L=/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute; LP=/home/benjamin/Documents/Repositories/working/LiDAR/.venv/bin/python
lidar() { # <benchmark> <cap_s>
  [ -f "$R/results/STOP" ] && { say "STOP: skipped lidar_p300 $1"; return; }
  local out="$R/results/$1/lidar_p300/$(date +%Y%m%d-%H%M%S)"; mkdir -p "$out" "$L/result/LiDAR/x200"
  local start=$(date +%s)
  (cd "$L" && timeout -s TERM -k 30 "$2" "$LP" main/picroute.py --benchmark=benchmarks/$1/$1.yml --config=config/comp_LiDAR_x200.yml --run.output_layout_gds_path=result/LiDAR/x200/${1}_comp_LiDAR_x200.gds > "$out/run.log" 2>&1 < /dev/null)
  local rc=$?; local wall=$(( $(date +%s) - start ))
  local it; it=$(grep -o "DrGridRoute: Iteration: *[0-9]*" "$out/run.log" | tail -1 | grep -o "[0-9]*$")
  local agg; agg=$(grep -o -E "Net: n_[0-9]+, WL: [0-9.]+ um, Accumulated bend: [0-9.]+,Crossing: [0-9]+, insertion loss: [0-9.]+, DRV: [0-9]+" "$out/run.log" | awk -F'[:,]' '{n++; wl+=$4; x+=$8; if($12+0>0) drv++} END{printf "nets=%d WL_um=%.1f physical_crossings=%d DRV_nets=%d", n, wl, x/2, drv}')
  echo "config=lidar_p300 benchmark=$1 rc=$rc wall=${wall}s iterations=$it $agg" > "$out/run.txt"
  cp "$L/result/LiDAR/x200/${1}_comp_LiDAR_x200.gds" "$out/" 2>/dev/null
  say "lidar_p300 $1 | $(cat "$out/run.txt")"
}
for b in benes_8x8 multiportmmi_8x8 benes_16x16 multiportmmi_16x16 multiportmmi_32x32; do lidar $b 3600; done
lidar benes_32x32 14400
lidar multiportmmi_64x64 14400
say "night queue done"
