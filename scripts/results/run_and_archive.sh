#!/bin/bash
# run_and_archive.sh <config> <benchmark> [routing_flow args...]
#
# Owner rule 2026-09-16: every benchmark run that may feed the paper is stored
# separately per configuration, so waveguide lengths, crossing counts and every
# other metric can be re-read later without rerunning. <config> is a short
# label chosen by the caller (e.g. lidar-pure, contribution1, contribution2,
# lidar-pure_span). The run gets a private working directory (symlinks to the
# repository, own build/), so parallel runs never share outputs; afterwards the
# archive directory results/<benchmark>/<config>/<timestamp>/ receives the
# routed GDS, both verification JSONs, the run log, a status line and the
# metrics summary from scripts/results/summarize_run.py.
set -u
config="$1"; bench="$2"; shift 2
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
stamp="$(date +%Y%m%d-%H%M%S)"
out="$R/results/$bench/$config/$stamp"; mkdir -p "$out"
wd="$(mktemp -d "${TMPDIR:-/tmp}/run_${bench}_${config}_XXXX")"
mkdir -p "$wd/build"
for e in "$R"/* "$R"/.venv; do b="$(basename "$e")"; [ "$b" = build ] && continue; [ "$b" = results ] && continue; ln -s "$e" "$wd/$b"; done
echo "config=$config benchmark=$bench args=$* start=$(date -Is) commit=$(git -C "$R" rev-parse --short HEAD) env_negotiated=${PHOTONIC_ROUTER_NEGOTIATED_REPAIR:-unset}" > "$out/run.txt"
start=$(date +%s)
(cd "$wd" && PYTHONUNBUFFERED=1 "$R/.venv/bin/python" routing_flow.py "$bench" "$@" > "$out/run.log" 2>&1 < /dev/null)
rc=$?; wall=$(( $(date +%s) - start ))
echo "rc=$rc wall=${wall}s end=$(date -Is)" >> "$out/run.txt"
cp "$wd"/build/verification/"$bench"_*_verification.json "$out/" 2>/dev/null
cp "$wd"/build/routed_"$bench"*.gds "$out/" 2>/dev/null
"$R/.venv/bin/python" "$R/scripts/results/summarize_run.py" "$out" >> "$out/run.txt" 2>/dev/null
rm -rf "$wd"
cat "$out/run.txt"
