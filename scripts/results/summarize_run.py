"""Metrics of one archived run (see run_and_archive.sh): route count, total
and mean waveguide length, crossing count, verifier error counts.

Usage: summarize_run.py <archive dir>  -- prints one `metrics ...` line and
writes metrics.json next to the reports. Reads only the two verification
JSONs, so it also works on any build/verification/ pair.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def summarize(archive: Path) -> dict:
    photonic = next(archive.glob("*_photonic_verification.json"), None)
    crossing = next(archive.glob("*_crossing_verification.json"), None)
    out: dict = {}
    if photonic is not None:
        p = json.loads(photonic.read_text())
        out["photonic_error_count"] = p.get("error_count")
        out["routed_record_count"] = p.get("routed_record_count")
        out["expected_route_count"] = p.get("expected_route_count")
    if crossing is not None:
        c = json.loads(crossing.read_text())
        m = c.get("metrics", {})
        out["crossing_error_count"] = c.get("error_count")
        out["crossing_count"] = m.get("crossing_count")
        out["illegal_crossing_count"] = m.get("illegal_crossing_count")
        out["total_physical_insertion_loss"] = m.get("total_physical_insertion_loss")
        costs = c.get("route_costs") or []
        lengths = [float(r["length_um"]) for r in costs if r.get("length_um") is not None]
        if lengths:
            out["route_cost_count"] = len(lengths)
            out["total_length_um"] = round(sum(lengths), 3)
            out["mean_length_um"] = round(sum(lengths) / len(lengths), 3)
            out["max_length_um"] = round(max(lengths), 3)
        crossings_per_net: dict[str, int] = {}
        for x in c.get("crossings") or []:
            for key in ("net_name_a", "net_name_b"):
                name = x.get(key)
                if name is not None:
                    crossings_per_net[name] = crossings_per_net.get(name, 0) + 1
        if crossings_per_net:
            out["max_crossings_per_net"] = max(crossings_per_net.values())
            out["nets_with_crossings"] = len(crossings_per_net)
    (archive / "metrics.json").write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    return out


def main() -> int:
    archive = Path(sys.argv[1])
    out = summarize(archive)
    print("metrics " + " ".join(f"{k}={v}" for k, v in sorted(out.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
