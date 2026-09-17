"""Markdown table over the archived runs in results/: latest run per
(benchmark, config). Columns: rc, wall, crossings, verifier errors, total
waveguide length (contribution 2: routes plus the placed structures), mean length, search attempts/failures/repairs, negotiated
rounds and rip-ups. Usage: table.py [results_dir]"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ORDER = ["benes_4x4", "multiportmmi_8x8", "benes_8x8", "multiportmmi_16x16", "benes_16x16",
         "multiportmmi_32x32", "benes_32x32", "multiportmmi_64x64", "benes_64x64",
         "benes_128x128", "multiportmmi_128x128"]


def latest_runs(root: Path):
    for bench_dir in sorted(root.iterdir()):
        if not bench_dir.is_dir():
            continue
        for cfg_dir in sorted(bench_dir.iterdir()):
            if not cfg_dir.is_dir():
                continue
            stamps = sorted(d for d in cfg_dir.iterdir() if d.is_dir())
            if stamps:
                yield bench_dir.name, cfg_dir.name, stamps[-1]


def read(run: Path) -> dict:
    out: dict = {}
    txt = (run / "run.txt").read_text() if (run / "run.txt").exists() else ""
    m = re.search(r"rc=(\d+)", txt)
    out["rc"] = m.group(1) if m else "?"
    m = re.search(r"wall=(\d+)s", txt)
    out["wall_s"] = int(m.group(1)) if m else None
    if (run / "metrics.json").exists():
        out.update(json.loads((run / "metrics.json").read_text()))
    for key, pat in (("crossing_count", r"physical_crossings=(\d+)"), ("total_length_um", r"WL_um=([0-9.]+)"),
                     ("lidar_iterations", r"iterations=(\d+)"), ("routed_record_count", r"nets=(\d+)"),
                     ("drv_nets", r"DRV_nets=(\d+)")):
        m = re.search(pat, txt)
        if m and key not in out:
            out[key] = float(m.group(1)) if "." in m.group(1) else int(m.group(1))
    return out


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "results")
    rows = [(b, c, run, read(run)) for b, c, run in latest_runs(root)]
    rows.sort(key=lambda r: (ORDER.index(r[0]) if r[0] in ORDER else 99, r[1]))
    print("| benchmark | config | rc | wall s | crossings | errors | total length um | mean um | attempts | fail | repairs | rounds | rip-ups g/l |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for b, c, run, m in rows:
        err = m.get("photonic_error_count", m.get("drv_nets", "?"))
        if "crossing_error_count" in m:
            err = f"{m.get('photonic_error_count')}/{m.get('crossing_error_count')}"
        rip = ""
        if "negotiated_global_ripups" in m:
            rip = f"{m['negotiated_global_ripups']}/{m['negotiated_local_ripups']}"
        tl = m.get("total_length_with_structures_um", m.get("total_length_um"))
        print(f"| {b} | {c} | {m['rc']} | {m.get('wall_s','')} | {m.get('crossing_count','')} | {err} | "
              f"{round(tl) if isinstance(tl,(int,float)) else ''} | {m.get('mean_length_um','')} | "
              f"{m.get('search_attempts','')} | {m.get('search_failures','')} | {m.get('search_repairs','')} | "
              f"{m.get('negotiated_rounds', m.get('lidar_iterations',''))} | {rip} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
