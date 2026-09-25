#!/bin/bash
# compare_verification_reports.sh <root_a> <root_b>
#
# Byte-compares every *_photonic_verification.json and
# *_crossing_verification.json archived under two results roots
# (<root>/<benchmark>/<config>/<timestamp>/), pairing the newest run of each
# benchmark/config in <root_a> with the newest in <root_b>. The verifier's
# reports carry no timing, so identical bytes mean identical verdicts and
# identical issue lists; this is the acceptance of
# .agent/execplans/2026-09-24-verifier-speed-on-large-layouts.md.
# Exit 0 when every paired report is identical and no report is missing.
set -u
a="$1"; b="$2"; rc=0; n=0
for cfg_dir in "$a"/*/*/; do
  bench="$(basename "$(dirname "$cfg_dir")")"; cfg="$(basename "$cfg_dir")"
  run_a="$(ls -d "$cfg_dir"*/ 2>/dev/null | sort | tail -1)"
  run_b="$(ls -d "$b/$bench/$cfg"/*/ 2>/dev/null | sort | tail -1)"
  if [ -z "$run_b" ]; then echo "MISSING in $b: $bench/$cfg"; rc=1; continue; fi
  for f in "$run_a"*_verification.json; do
    [ -e "$f" ] || continue
    name="$(basename "$f")"; n=$((n+1))
    if [ ! -e "$run_b/$name" ]; then echo "MISSING in $b: $bench/$cfg/$name"; rc=1
    elif ! cmp -s "$f" "$run_b/$name"; then echo "DIFFERENT: $bench/$cfg/$name"; rc=1
    fi
  done
done
echo "compared $n reports; $([ $rc -eq 0 ] && echo all identical || echo differences above)"
exit $rc
