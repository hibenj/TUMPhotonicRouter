#!/bin/bash
# Experiment 1 driver (2026-10-29, instrumentation; see
# .agent/TUMPhotonicRouter/experiments/01_instrumentation.md). Sequential:
#   phase 0  the nine-cell gate with every switch off (GATE_KEEP=1, root kept)
#   phase 1  the nine cells + benes_16x16 lidar-pure under
#            PHOTONIC_ROUTER_EXP_KERNEL_PROFILE=1 --debug-timing true --verbose-routes
#   phase 2  the nine cells under --net-order random / topological-random,
#            five seeds each (PHOTONIC_ROUTER_EXP_NET_ORDER_SEED)
# Archives land under results_exp1/ in the worktree; results_exp1/driver.log
# gets one line per run. Run from anywhere; needs the built kernel.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$R" || exit 1
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
ROOT="$R/results_exp1"; mkdir -p "$ROOT"
log="$ROOT/driver.log"; say() { echo "$(date -Is) $*" >> "$log"; }
run() { # <root> <config> <bench> <cap_s> <args...>
  local root="$1" cfg="$2" bench="$3" cap="$4"; shift 4
  local out; out=$(RESULTS_ROOT="$root" timeout -s TERM -k 30 "$cap" scripts/results/run_and_archive.sh "$cfg" "$bench" "$@" 2>&1 | tail -2 | tr '\n' ' ' | cut -c1-400)
  say "$root $cfg $bench | $out"
}
cell_args() { # <config> -> the gate's arguments for that configuration
  case "$1" in
    lidar-pure) echo "--crossing-mode lidar-pure" ;;
    contribution1) echo "--crossing-mode lidar-guided" ;;
    contribution2) echo "--preplaced-crossing-grids true --verbose-routes" ;;
  esac
}
BENCHES="benes_4x4_flat benes_8x8_flat multiportmmi_8x8"
CONFIGS="lidar-pure contribution1 contribution2"
phases="${1:-0 1 2}"

for phase in $phases; do
case "$phase" in
0)
  say "phase 0: gate, switches off"
  GATE_KEEP=1 scripts/results/gate_short.sh > "$ROOT/gate_off.txt" 2>&1
  say "phase 0 done rc=$? ($(tail -1 "$ROOT/gate_off.txt"))"
  ;;
1)
  say "phase 1: kernel profile"
  export PHOTONIC_ROUTER_EXP_KERNEL_PROFILE=1
  for b in $BENCHES; do for c in $CONFIGS; do
    run "$ROOT/profile" "$c" "$b" 900 $(cell_args "$c") --debug-timing true --verbose-routes
  done; done
  run "$ROOT/profile" lidar-pure benes_16x16_flat 900 --crossing-mode lidar-pure --debug-timing true --verbose-routes
  unset PHOTONIC_ROUTER_EXP_KERNEL_PROFILE
  say "phase 1 done"
  ;;
2)
  say "phase 2: net-order sensitivity"
  for policy in random topological-random; do for seed in 1 2 3 4 5; do
    export PHOTONIC_ROUTER_EXP_NET_ORDER_SEED=$seed
    for b in $BENCHES; do for c in $CONFIGS; do
      run "$ROOT/order/$policy/seed$seed" "$c" "$b" 300 $(cell_args "$c") --net-order "$policy"
    done; done
  done; done
  unset PHOTONIC_ROUTER_EXP_NET_ORDER_SEED
  say "phase 2 done"
  ;;
esac
done
say "all done"
