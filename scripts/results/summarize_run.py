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
    out.update(log_metrics(archive / "run.log"))
    (archive / "metrics.json").write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    return out


_LOG_PATTERNS = {
    # `route search: astar_loop=2201.9764s, attempts=1077, failures=62, simple=465/895, repairs=26, deferred=0`
    "search_astar_loop_s": r"route search: astar_loop=([0-9.]+)s",
    "search_attempts": r"route search: .*?attempts=(\d+)",
    "search_failures": r"route search: .*?failures=(\d+)",
    "search_repairs": r"route search: .*?repairs=(\d+)",
    "search_deferred": r"route search: .*?deferred=(\d+)",
    "search_simple_routes": r"route search: .*?simple=(\d+)/",
    # `native_negotiated_done t=1136.4 rounds=3 global_ripups=1 local_ripups=17 ...` (PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG)
    "negotiated_rounds": r"native_negotiated_done .*?rounds=(\d+)",
    "negotiated_global_ripups": r"native_negotiated_done .*?global_ripups=(\d+)",
    "negotiated_local_ripups": r"native_negotiated_done .*?local_ripups=(\d+)",
    "negotiated_probe_guided": r"native_negotiated_done .*?probe_guided=(\d+)",
    "negotiated_crossing_free": r"native_negotiated_done .*?crossing_free=(\d+)",
    "negotiated_wall_s": r"native_negotiated_done t=([0-9.]+)",
    # contribution 2: `Grids: 209 placed, 209 crossing component(s) ... (7.63s)`
    "preplaced_grid_count": r"Grids: (\d+) placed",
    "preplaced_crossing_components": r"Grids: \d+ placed, (\d+) crossing component",
    "preplaced_build_s": r"Grids: .*?\(([0-9.]+)s\)",
}


def log_metrics(log_path: Path) -> dict:
    """Search-effort counters from the run log: attempts, failures, repairs,
    A* loop seconds (every run), the negotiated engine's rounds and rip-ups
    (runs with PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG), contribution 2's grid
    counts and structure build time. Counters that occur several times
    (several routing batches) are summed; seconds too."""
    import re

    if not log_path.exists():
        return {}
    text = log_path.read_text(errors="replace")
    out: dict = {}
    for key, pattern in _LOG_PATTERNS.items():
        values = re.findall(pattern, text)
        if not values:
            continue
        if key.endswith("_s"):
            out[key] = round(sum(float(v) for v in values), 3)
        elif key in ("negotiated_rounds", "preplaced_grid_count", "preplaced_crossing_components"):
            out[key] = max(int(v) for v in values)
        else:
            out[key] = sum(int(v) for v in values)
    braids = len(re.findall(r"native_repair_braid_result .*?keep=true", text))
    if braids:
        out["braid_repairs_kept"] = braids
    return out


def main() -> int:
    archive = Path(sys.argv[1])
    out = summarize(archive)
    print("metrics " + " ".join(f"{k}={v}" for k, v in sorted(out.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
