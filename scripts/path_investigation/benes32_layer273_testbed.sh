#!/bin/bash
# Testbed for the benes_32x32 last-stage blocker (net 273): routes only nets 257-272
# (the 16 nets of net 273's layer that precede it, source x=3864) under the stable flags
# + LSC weight 1.0 and stops. 47 s; produces exactly the corridor geometry of the full run
# (verified 2026-09-03 with scripts/path_investigation/gds_runs.py). Append 273 to the
# list and stop after 17 to reproduce the hang; add PHOTONIC_ROUTER_* diag env as needed.
cd /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter || exit 1
SP=${SP:-/tmp}
export PYTHONUNBUFFERED=1
export PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT=1.0
export PHOTONIC_ROUTER_NATIVE_PROGRESS=1
export PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS="257,258,259,260,261,262,263,264,265,266,267,268,269,270,271,272"
timeout 2400 .venv/bin/python routing_flow.py benes_32x32 --debug-stop-after-route 16 > "$SP/benes32_layer273_testbed.stdout.log" 2> "$SP/benes32_layer273_testbed.stderr.log" < /dev/null
rc=$?
cp build/routed_benes_32x32.gds build/routed_benes_32x32_E1_V1_layer_257_272.gds
echo "exit=$rc at $(date +%T)" > "$SP/benes32_layer273_testbed.status"
