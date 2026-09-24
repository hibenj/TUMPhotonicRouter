#!/bin/bash
# Two-band testbed for the multiportmmi_64x64 net-575 case (Milestone 4 finding, 2026-09-14 18:47:
# the probe route of net 575 crossed net 573 without a crossing event; the post-commit guard rejected
# it and the negotiated engine aborted instead of running its global round). Routes only net ids
# 448-575 (band 8: mmi0_ps -> mmi0_multiport, band 9: mmi0_multiport -> mol_array) in the stable
# lidar-pure order, hoisted to the front, stops after 128 routes. Same conventions as
# mm64_bands45_testbed.sh (OUT, 2 h cap, trace + progress on; PHOTONIC_ROUTER_NEGOTIATED_REPAIR=1
# used to select the new engine -- gone, the negotiated engine is the only one since Milestone 8).
# Verify the subset against a full --verbose-routes run before trusting it.
OUT="${OUT:-$PWD}"
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
export PYTHONUNBUFFERED=1
export PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1
export PHOTONIC_ROUTER_NATIVE_PROGRESS=1
export PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS="$(seq -s, 448 575)"
start=$(date +%s)
timeout 7200 .venv/bin/python routing_flow.py multiportmmi_64x64 --crossing-mode lidar-pure --verbose-routes --debug-stop-after-route 128 > "$OUT/mm64_bands89.stdout.log" 2> "$OUT/mm64_bands89.stderr.log" < /dev/null
rc=$?
echo "rc=$rc wall=$(( $(date +%s) - start ))s" > "$OUT/mm64_bands89.status"
