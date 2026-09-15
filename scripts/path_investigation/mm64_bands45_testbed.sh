#!/bin/bash
# Two-band testbed for the multiportmmi_64x64 lidar-pure repair failure (ExecPlan
# .agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md, Milestone 1).
# Routes only net ids 192-319 (bands 4 and 5: mmi0_ps -> mmi0_multiport -> mmi0_ps) in the
# stable lidar-pure order, hoisted to the front in that order, and stops after 128 routes.
# Verification of the subset (2026-09-14): the 128 `Routing [i/128] <name>` lines equal positions
# 192-319 of a full `--verbose-routes` run; job 309's native endpoints are (2425, 2769) -> (3130, 2673).
# Acceptance of the testbed: the stderr trace contains
#   native_repair_source_layer_center_out_start ... trigger_net=313 victim=231
# i.e. the same trigger as the full 4 h run. 2 h cap. Logs go to $OUT (default: the invocation dir).
# Optional: PHOTONIC_ROUTER_NEGOTIATED_REPAIR=1 in the environment selects the negotiated engine.
OUT="${OUT:-$PWD}"
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
export PYTHONUNBUFFERED=1
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
export PHOTONIC_ROUTER_NATIVE_PROGRESS=1
export PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS="$(seq -s, 192 319)"
start=$(date +%s)
timeout 7200 .venv/bin/python routing_flow.py multiportmmi_64x64 --crossing-mode lidar-pure --verbose-routes --debug-stop-after-route 128 > "$OUT/mm64_bands45.stdout.log" 2> "$OUT/mm64_bands45.stderr.log" < /dev/null
rc=$?
echo "rc=$rc wall=$(( $(date +%s) - start ))s" > "$OUT/mm64_bands45.status"
