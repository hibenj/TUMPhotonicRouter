"""Results table of the DATE paper from the archived runs (results/).

Usage: paper_table.py [results_dir] [main.tex]
Prints the LaTeX table; with main.tex given, replaces the table*
environment labelled tab:routing-results in place.

Layout (owner 2026-09-17, following the fcn paper convention of absolute
values plus Delta columns and one mean row): per benchmark the absolute
lidar-pure numbers, then for each contribution the absolute values plus
the relative change of A* time and waveguide length against lidar-pure in
percent (negative = better); the last row is the geometric mean of the
ratios over the benchmarks from 8x8 upwards that all three configurations
complete. Repairs and crossings stay absolute. `--` = not run or not
completed.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from table import latest_runs, read  # noqa: E402

BENCH = [
    ("benes_4x4", "Benes $4\\times4$"), ("benes_8x8", "Benes $8\\times8$"),
    ("benes_16x16", "Benes $16\\times16$"), ("benes_32x32", "Benes $32\\times32$"),
    ("benes_64x64", "Benes $64\\times64$"), ("benes_128x128", "Benes $128\\times128$"),
    ("multiportmmi_8x8", "ADEPT $8\\times8$"), ("multiportmmi_16x16", "ADEPT $16\\times16$"),
    ("multiportmmi_32x32", "ADEPT $32\\times32$"), ("multiportmmi_64x64", "ADEPT $64\\times64$"),
    ("multiportmmi_128x128", "ADEPT $128\\times128$"),
]
# Owner decision 2026-09-19: the Benes comparison routes the netlist LiDAR routes
# (switches expanded into their primitives plus the switch-internal nets,
# benchmarks/benes_<n>x<n>_flat.py); our cells of a Benes row come from the
# `_flat` archives, LiDAR's from the benchmark it was given.
OURS_SOURCE = {b: b + "_flat" for b, _ in BENCH if b.startswith("benes_")}
MEAN_EXCLUDED = {"benes_4x4"}  # 0.02 s A* times give meaningless ratios
# Manual DRV corrections for LiDAR rows (owner inspection in KLayout, 2026-09-18):
# LiDAR's own DRV counter does not check the die boundary. ADEPT 8x8 at the
# matched price routes four output nets (n_105, n_106, n_108, n_110) outside
# the die, to the right of the grating-coupler array, and thereby avoids
# three of the 33 planned crossings (30 realized). That is one illegal
# route region -> DRV 1.
LIDAR_DRV_OVERRIDES = {("multiportmmi_8x8", "lidar_p300"): "1"}


def ok(m: dict | None) -> bool:
    return (m is not None and m.get("rc") == "0" and m.get("routing_loop_s") is not None
            and m.get("gds_length_um") is not None)


def routing_time(m: dict) -> float:
    """Routing time of ours (owner rule 2026-09-19: the actual routing time on
    both sides): the routing loop = obstacle map + search with rip-up and
    repairs, plus contribution 2's structure construction (`routing_loop_s`
    from summarize_run.py). LiDAR's counterpart is `lidar_routing_loop_s`,
    its loop from `Start Detailed Routing` to `DrGridRoute succeed/fail`;
    the `routing_time_s` / `lidar_detailed_routing_s` pair adds geometry
    realization and evaluation on both sides (< 1 %)."""
    return float(m["routing_loop_s"])


def astar(m: dict) -> float:
    return float(m["search_astar_loop_s"])


def length_mm(m: dict) -> float:
    """Waveguide length measured on the routed GDS (routes + crossing elements,
    `gds_length_um` from summarize_run.py; owner rule 2026-09-19: one length
    measure for every configuration)."""
    return float(m["gds_length_um"]) / 1000.0


def fmt_t(x: float) -> str:
    return f"{x:.1f}" if x >= 1 else f"{x:.2f}"


def fmt_pct(ratio: float) -> str:
    pct = (ratio - 1.0) * 100.0
    return f"$\\SI{{{pct:+.1f}}}{{\\%}}$" if abs(pct) < 99.95 else f"$\\SI{{{pct:+.2f}}}{{\\%}}$"


def build(results: Path) -> tuple[str, dict]:
    rows = {}
    for b, c, run in latest_runs(results):
        if c in ("lidar-pure", "contribution1", "contribution2", "lidar_p300", "lidar_p300_12h"):
            rows[(b, c)] = read(run) | {"archive": str(run)}
    lines: list[str] = []
    logs: dict[str, list[float]] = {"c1_t": [], "c1_l": [], "c2_t": [], "c2_l": []}
    sources: dict[str, dict] = {}

    def lidar_row(b: str) -> dict | None:
        # matched-price LiDAR: the 12 h archive when it completed, else the 4 h one
        for c in ("lidar_p300_12h", "lidar_p300"):
            m = rows.get((b, c))
            if m is not None and m.get("rc") == "0" and m.get("routed_record_count") and m.get("lidar_routing_loop_s") is not None:
                return m | {"config": c}
        return None

    def source(m: dict | None, config: str) -> dict:
        if m is None:
            return {"archive": None}
        keys = ("routing_loop_s", "lidar_routing_loop_s", "preplaced_build_s", "gds_length_um", "crossing_count", "drv_nets")
        return {"archive": m.get("archive"), "config": config} | {k: m.get(k) for k in keys if m.get(k) is not None}

    for b, label in BENCH:
        o = OURS_SOURCE.get(b, b)
        lidar = lidar_row(b)
        p = rows.get((o, "lidar-pure")); c1 = rows.get((o, "contribution1")); c2 = rows.get((o, "contribution2"))
        sources[b] = {"ours_benchmark": o, "lidar": source(lidar, lidar["config"] if lidar else ""),
                      "baseline": source(p, "lidar-pure"), "contribution1": source(c1, "contribution1"),
                      "contribution2": source(c2, "contribution2")}
        cells = [label]
        if lidar is not None:
            drv = LIDAR_DRV_OVERRIDES.get((b, lidar["config"]), str(lidar.get("drv_nets", "")))
            cells += [fmt_t(float(lidar["lidar_routing_loop_s"])), str(lidar.get("crossing_count", "")), drv]
        else:
            cells += ["--"] * 3
        cells += [fmt_t(routing_time(p)), f"{length_mm(p):.1f}", str(p.get("crossing_count", ""))] if ok(p) else ["--"] * 3
        for key, m in (("c1", c1), ("c2", c2)):
            if not ok(m):
                cells += ["--"] * (5 if key == "c1" else 6)
                continue
            dt = fmt_pct(routing_time(m) / routing_time(p)) if ok(p) else "--"
            dl = fmt_pct(length_mm(m) / length_mm(p)) if ok(p) else "--"
            cells += [fmt_t(routing_time(m))]
            if key == "c2":
                cells += [fmt_t(float(m.get("preplaced_build_s", 0.0)))]
            cells += [dt, f"{length_mm(m):.1f}", dl, str(m.get("crossing_count", ""))]
        lines.append("            " + " & ".join(cells) + " \\\\")
        if b == "benes_128x128":
            lines.append("            \\midrule")
        if ok(p) and ok(c1) and ok(c2) and b not in MEAN_EXCLUDED:
            logs["c1_t"].append(math.log(routing_time(c1) / routing_time(p))); logs["c1_l"].append(math.log(length_mm(c1) / length_mm(p)))
            logs["c2_t"].append(math.log(routing_time(c2) / routing_time(p))); logs["c2_l"].append(math.log(length_mm(c2) / length_mm(p)))
    gm = {k: math.exp(sum(v) / len(v)) for k, v in logs.items()}
    n = len(logs["c1_t"])
    mean_row = ("            \\emph{Geometric mean} & & & & & & & & " + fmt_pct(gm["c1_t"]) + " & & " + fmt_pct(gm["c1_l"])
                + " & & & & " + fmt_pct(gm["c2_t"]) + " & & " + fmt_pct(gm["c2_l"]) + " & \\\\")
    table = r"""\begin{table*}[!t]
    \caption{Routing results for public LiDAR, our baseline, and both contributions. $t$ is routing time, $t_\mathrm{build}$ is the structure-construction portion included in Contribution~2's $t$, and $L$ is the total waveguide length measured on the routed layout (routed waveguides and crossing elements, including placed structures). Cross.\ counts realized crossings, and DRV counts nets with DRVs. $\Delta$ values are relative to the baseline.}
    \label{tab:routing-results}
    \centering
    \scriptsize
    \setlength{\tabcolsep}{1.5pt}
    \renewcommand{\arraystretch}{0.9}
    \begin{adjustbox}{max width=\textwidth}
        \begin{tabular}{@{}lrrrrrrrrrrrrrrrrr@{}}
            \toprule
            & \multicolumn{3}{c}{\textsc{LiDAR}~\cite{zhou2025lidar}} & \multicolumn{3}{c}{\textsc{Baseline}} & \multicolumn{5}{c}{\textsc{Contribution 1}} & \multicolumn{6}{c}{\textsc{Contribution 2}} \\\\
            \cmidrule(lr){2-4} \cmidrule(lr){5-7} \cmidrule(lr){8-12} \cmidrule(l){13-18}
            Benchmark & $t$ [s] & Cross. & DRV & $t$ [s] & $L$ [mm] & Cross. & $t$ [s] & $\Delta t$ & $L$ [mm] & $\Delta L$ & Cross. & $t$ [s] & $t_\mathrm{build}$ [s] & $\Delta t$ & $L$ [mm] & $\Delta L$ & Cross. \\\\
            \midrule
%s
            \midrule
%s
            \bottomrule
        \end{tabular}
    \end{adjustbox}
\end{table*}""" % ("\n".join(lines), mean_row)
    meta = {
        "definition": "t = routing_loop_s (net routing phase: obstacle map + search with rip-up and repair; contribution 2 plus preplaced_build_s, shown as t_build); LiDAR = lidar_routing_loop_s (Start Detailed Routing -> DrGridRoute succeed/fail); L = gds_length_um (routed polygons + crossing* instances on the WG layer of the routed GDS, area / 0.5 um)",
        "benes_rows": "benes_<n>x<n>_flat archives (switches expanded into their primitives plus the internal nets, the netlist LiDAR routes) on the frozen engine 8ddde83",
        "means": {"n": n, **gm},
        "sources": sources,
    }
    return table, meta


def main() -> int:
    results = Path(sys.argv[1] if len(sys.argv) > 1 else "results")
    table, gm = build(results)
    if len(sys.argv) > 2:
        tex = Path(sys.argv[2]); s = tex.read_text()
        start = s.index("\\begin{table*}[!t]"); end = s.index("\\end{table*}", start) + len("\\end{table*}")
        assert "tab:routing-results" in s[start:end]
        tex.write_text(s[:start] + table + s[end:])
        (tex.parent / "EXPERIMENTS_TABLE_SOURCES.json").write_text(json.dumps(gm, indent=2) + "\n")
    print(table)
    m = gm["means"]
    print("%% geometric means over %d benchmarks: C1 t %.3f L %.4f | C2 t %.4f L %.4f" % (m["n"], m["c1_t"], m["c1_l"], m["c2_t"], m["c2_l"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
