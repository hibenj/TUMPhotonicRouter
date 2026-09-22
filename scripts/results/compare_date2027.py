"""Compare a reproduction of the DATE 2027 table (archives written by
scripts/results/reproduce_date2027.sh) with the paper's sources, cell by cell.

Usage: compare_date2027.py <results_root> <EXPERIMENTS_TABLE_SOURCES.json>

For every cell of ours (baseline, contribution1, contribution2) that the
paper sources name, the latest run under <results_root>/<benchmark>/<config>
is read (scripts/results/table.py) and compared on the reproducible
quantities: crossing count, GDS waveguide length (um, exact to 0.01), verifier
errors (must be 0), and rc. Routing-loop time is printed as a ratio only:
it varies with the machine and is not part of the comparison. Cells without
a reproduction run are listed as missing. Exit code 1 on any mismatch."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from table import latest_runs, read  # noqa: E402

CONFIGS = {"baseline": "lidar-pure", "contribution1": "contribution1", "contribution2": "contribution2"}
LENGTH_TOL_UM = 0.01


def main() -> int:
    root = Path(sys.argv[1])
    sources = json.loads(Path(sys.argv[2]).read_text())["sources"]
    runs = {(b, c): run for b, c, run in latest_runs(root)}
    print("| benchmark | config | crossings paper/repro | GDS length um paper/repro | errors | t_loop ratio | status |")
    print("|---|---|---|---|---|---|---|")
    bad = 0
    for bench, cells in sources.items():
        ours_bench = cells.get("ours_benchmark", bench)
        for key, config in CONFIGS.items():
            cell = cells.get(key) or {}
            if cell.get("archive") is None:
                continue
            run = runs.get((ours_bench, config))
            if run is None:
                print(f"| {ours_bench} | {config} | {cell.get('crossing_count')} / -- | {cell.get('gds_length_um')} / -- | -- | -- | MISSING |")
                bad += 1
                continue
            m = read(run)
            if m.get("rc") == "?":  # run.txt has no rc line yet: still running or killed
                print(f"| {ours_bench} | {config} | {cell.get('crossing_count')} / -- | {cell.get('gds_length_um')} / -- | -- | -- | RUNNING ({run.name}) |")
                bad += 1
                continue
            problems = []
            if m.get("rc") != "0":
                problems.append(f"rc={m.get('rc')}")
            if int(m.get("photonic_error_count", -1)) != 0:
                problems.append(f"errors={m.get('photonic_error_count')}")
            if m.get("crossing_count") != cell.get("crossing_count"):
                problems.append("crossings")
            length = m.get("gds_length_um")
            if length is None or abs(float(length) - float(cell["gds_length_um"])) > LENGTH_TOL_UM:
                problems.append("length")
            t_ratio = (float(m["routing_loop_s"]) / float(cell["routing_loop_s"])
                       if m.get("routing_loop_s") and cell.get("routing_loop_s") else None)
            status = "ok" if not problems else "MISMATCH " + ",".join(problems)
            bad += bool(problems)
            print(f"| {ours_bench} | {config} | {cell.get('crossing_count')} / {m.get('crossing_count')} | "
                  f"{cell.get('gds_length_um')} / {length} | {m.get('photonic_error_count')} | "
                  f"{t_ratio:.2f} | {status} |" if t_ratio is not None else
                  f"| {ours_bench} | {config} | {cell.get('crossing_count')} / {m.get('crossing_count')} | "
                  f"{cell.get('gds_length_um')} / {length} | {m.get('photonic_error_count')} | -- | {status} |")
    print(f"\n{bad} cell(s) missing or mismatched" if bad else "\nall reproduced cells match")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
