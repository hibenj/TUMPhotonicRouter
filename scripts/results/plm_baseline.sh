#!/bin/bash
# Phase 6 slice 0: the path-length-matching (PLM) baseline. Routes
# heater_s_mod and the twelve cascades once each under their benchmark
# stable flags (STABLE_ROUTING_FLAGS in benchmarks/<bench>.py: 90-degree
# turns, 10 um bends, PLM with output matching, heater obstacles, electrical
# routing, 3 um clearance, no crossings; `routing_flow.py <bench>` applies
# them itself) and archives every run under RESULTS_ROOT (default
# results_plm/) with the config label `plm`. The archive (run.log with the
# SUMMARY line, run.txt with the metrics line, the routed GDS whose kfactory
# meta info carries the flow's path_length_analysis / meander_insertion_report
# / electrical_routing results, the verification JSON) is what
# scripts/results/plm_table.py reads; docs/PLM_REPRODUCTION.md has the rest.
#
# Usage: plm_baseline.sh [all|<bench>...]      (default all, sequential)
#        plm_baseline.sh --list                 prints the benchmark names
# One benchmark per call is the unit, so a machine with cores to spare runs
#   parallel -j 13 scripts/results/plm_baseline.sh ::: $(scripts/results/plm_baseline.sh --list)
# Environment: RESULTS_ROOT (archive root), PLM_TIMEOUT_S (per run, default
# 3600), and `touch $RESULTS_ROOT/STOP` skips every run not yet started.
# The log is $RESULTS_ROOT/reproduce.txt, one line per run (like
# reproduce_date2027.sh); the archive's `metrics` line is appended to it.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$R" || exit 1
PLM_BENCHMARKS="heater_s_mod"
for n in 8 12 16; do
  for layout in compact diagonal folded folded_compact; do
    PLM_BENCHMARKS="$PLM_BENCHMARKS cascade_${n}_${layout}"
  done
done
if [ "${1:-}" = "--list" ]; then echo "$PLM_BENCHMARKS" | tr ' ' '\n'; exit 0; fi
export RESULTS_ROOT="${RESULTS_ROOT:-$R/results_plm}"; mkdir -p "$RESULTS_ROOT"
timeout_s="${PLM_TIMEOUT_S:-3600}"
log="$RESULTS_ROOT/reproduce.txt"; say() { echo "$(date -Is) $*" >> "$log"; }
one() { # <benchmark>
  [ -f "$RESULTS_ROOT/STOP" ] && { say "STOP: skipped plm $1"; return; }
  local out
  out=$(timeout -s TERM -k 30 "$timeout_s" scripts/results/run_and_archive.sh plm "$1" 2>&1 | tail -2 | tr '\n' ' ')
  say "plm $1 | $out"
}
if [ $# -eq 0 ] || [ "$1" = all ]; then set -- $PLM_BENCHMARKS; fi
say "PLM baseline ($*) start commit=$(git rev-parse --short HEAD) host=$(hostname) kernel=$(stat -c %y python/photonic_router/_rust.abi3.so | cut -c1-19)"
for b in "$@"; do
  case " $PLM_BENCHMARKS " in
    *" $b "*) one "$b" ;;
    *) say "unknown benchmark $b (not in the PLM set); skipped" ;;
  esac
done
say "PLM baseline ($*) done"
