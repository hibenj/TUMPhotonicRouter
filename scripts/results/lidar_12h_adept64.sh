#!/bin/bash
# LiDAR reruns of the time-limited cases with a 12 h cap (owner go 2026-09-17 16:40), all three in parallel:
#   price 0  benes_32x32 (was 1 h timeout), price 300 benes_32x32 (was 4 h), price 300 multiportmmi_64x64 (was 4 h, peak 17 GB).
# Memory watchdog: below 2 GB MemAvailable the largest picroute process is killed and its run marked OOM.
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$R" || exit 1
L=/home/benjamin/Documents/Repositories/working/LiDAR/src/picroute; LP=/home/benjamin/Documents/Repositories/working/LiDAR/.venv/bin/python
log="$R/results/rerun_all.txt"; say() { echo "$(date -Is) $*" >> "$log"; }
lidar() { # <benchmark> <price_cfg: comp_LiDAR|comp_LiDAR_x200> <config label>
  local b="$1" cfg="$2" label="$3"
  local out="$R/results/$b/$label/$(date +%Y%m%d-%H%M%S)"; mkdir -p "$out" "$L/result/LiDAR/$label"
  local start=$(date +%s)
  (cd "$L" && timeout -s TERM -k 30 43200 "$LP" main/picroute.py --benchmark=benchmarks/$b/$b.yml --config=config/$cfg.yml --run.output_layout_gds_path=result/LiDAR/$label/${b}_${cfg}.gds > "$out/run.log" 2>&1 < /dev/null)
  local rc=$?; local wall=$(( $(date +%s) - start ))
  local it; it=$(grep -o "DrGridRoute: Iteration: *[0-9]*" "$out/run.log" | tail -1 | grep -o "[0-9]*$")
  local agg; agg=$(grep -o -E "Net: n_[0-9]+, WL: [0-9.]+ um, Accumulated bend: [0-9.]+,Crossing: [0-9]+, insertion loss: [0-9.]+, DRV: [0-9]+" "$out/run.log" | awk -F'[:,]' '{n++; wl+=$4; x+=$8; if($12+0>0) drv++} END{printf "nets=%d WL_um=%.1f physical_crossings=%d DRV_nets=%d", n, wl, x/2, drv}')
  local oom=""; [ -f "$out/OOM" ] && oom=" oom=$(cat "$out/OOM")"
  echo "config=$label benchmark=$b rc=$rc wall=${wall}s cap=43200s iterations=$it $agg$oom" > "$out/run.txt"
  cp "$L/result/LiDAR/$label/${b}_${cfg}.gds" "$out/" 2>/dev/null
  say "$label $b | $(cat "$out/run.txt")"
}
say "lidar 12 h rerun start: ADEPT 64 price 300 alone"
# (single run: ADEPT 64 only, owner 2026-09-18 05:40)

lidar multiportmmi_64x64 comp_LiDAR_x200 lidar_p300_12h &
# watchdog until all three are done
while pgrep -f "main/picroute.p[y]" > /dev/null; do
  avail=$(awk '/MemAvailable/ {printf "%d", $2/1024}' /proc/meminfo)
  echo "$(date -Is) avail_mb=$avail" >> "$R/results/lidar_12h_watchdog.trace"
  if [ "$avail" -lt 2048 ]; then
    pid=$(ps -eo pid,rss,args | grep "main/picroute.p[y]" | sort -k2 -rn | head -1 | awk '{print $1}')
    bench=$(ps -o args= -p "$pid" | grep -o "benchmarks/[a-z_0-9]*" | head -1 | cut -d/ -f2)
    cfg=$(ps -o args= -p "$pid" | grep -o "config/[A-Za-z_0-9]*" | head -1 | cut -d/ -f2)
    label=$([ "$cfg" = comp_LiDAR ] && echo lidar_p0_12h || echo lidar_p300_12h)
    out=$(ls -td "$R/results/$bench/$label"/2026*/ | head -1)
    echo "killed $(date -Is) avail_mb=$avail rss_kb=$(ps -o rss= -p "$pid")" > "$out/OOM"
    say "OOM watchdog: killing $bench $label (pid $pid, avail ${avail} MB)"
    kill -TERM "$pid"
  fi
  sleep 60
done
wait
say "lidar 12 h reruns done"
