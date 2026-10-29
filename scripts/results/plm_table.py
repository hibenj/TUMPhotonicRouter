"""The path-length-matching (PLM) baseline table (phase 6 slice 0).

Usage: plm_table.py <results_root> [--config plm] [--json <path>]
       [--compare <docs/plm_baseline.json>] [--only-present]

Reads the latest run per benchmark under <results_root>/<benchmark>/<config>
(archives of scripts/results/plm_baseline.sh via run_and_archive.sh) and
prints a Markdown table of the reproducible quantities: rc, verifier error
counts (photonic / electrical), the number of nets, the total routed
waveguide length measured on the GDS (um, summarize_run.py), the meander
length the flow inserted (um), the PLM group count, the groups over
tolerance, and the realized residual spread: the maximum over the matching
groups of (max - min) realized arrival length of the group's incoming
edges, plus the same for the output-coupler group. Routing-loop and PLM
phase times and the wall time are information only.

Where the numbers come from: run.txt (rc, wall) and metrics.json
(summarize_run.py: verifier errors, nets, GDS length, phase times); the
SUMMARY line of run.log (plm_groups, plm_over_tolerance, electrical verdict);
and the routed GDS, whose top-cell `kfactory:info` meta info carries the
flow's layout info as written by gdsfactory: `path_length_analysis`
(matching_group_diagnostics with realized_arrival_spread_um per group,
output_matching_diagnostics, path_length_acceptance), `meander_insertion_
report` (total_inserted_extra_length_um, exact) and `electrical_routing`
(verification_error_count). A run that failed before the GDS was written
(PLM acceptance raises, rc != 0) keeps its rc and whatever the log says
(`Path-length matching failed: ...`); its other cells are empty.

`--json <path>` writes the cells keyed by benchmark. `--compare <json>`
compares the root against that stored baseline on the exact quantities (rc,
errors, nets, groups, over-tolerance count; GDS length, meander length and
both spreads to 0.01 um) and exits 1 on any mismatch or missing benchmark
(`--only-present` ignores benchmarks the root has no run for, for the short
gate), printing `all PLM cells match` otherwise. Times are never compared.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from table import latest_runs, read

DEFAULT_CONFIG = "plm"
ORDER = ["heater_s_mod"] + [
    f"cascade_{n}_{layout}"
    for n in (8, 12, 16)
    for layout in ("compact", "diagonal", "folded", "folded_compact")
]
TOL_UM = 0.01
EXACT_KEYS = (
    "rc",
    "photonic_error_count",
    "electrical_error_count",
    "nets",
    "plm_groups",
    "plm_over_tolerance",
)
LENGTH_KEYS = ("gds_length_um", "meander_inserted_um", "group_spread_um", "output_spread_um")
INFO_KEYS = ("routing_loop_s", "plm_phase_s", "wall_s", "stamp", "commit", "note")


def _number(value: str) -> int | float | None:
    if value in ("-", ""):
        return None
    return float(value) if "." in value else int(value)


def log_cells(log_path: Path) -> dict:
    """The SUMMARY line, the PLM phase time and the PLM failure message of run.log."""
    out: dict = {}
    if not log_path.exists():
        return out
    text = log_path.read_text(errors="replace")
    summary = re.search(r"^SUMMARY (.*)$", text, re.MULTILINE)
    if summary is not None:
        fields = dict(part.split("=", 1) for part in summary.group(1).split() if "=" in part)
        out["nets"] = _number(fields.get("nets", "-"))
        out["plm_groups"] = _number(fields.get("plm_groups", "-"))
        out["plm_over_tolerance"] = _number(fields.get("plm_over_tolerance", "-"))
        out["meander_inserted_um"] = _number(fields.get("meander_inserted_um", "-"))
        out["electrical_ok"] = fields.get("electrical_ok")
        out["electrical_failed_wires"] = _number(fields.get("electrical_failed_wires", "-"))
    # `        - path-length matching phase: 1.2345 s (analysis=...)` (one per routing batch; summed)
    plm_phase = re.findall(r"path-length matching phase: ([0-9.]+) s", text)
    if plm_phase:
        out["plm_phase_s"] = round(sum(float(v) for v in plm_phase), 3)
    failure = re.search(r"Path-length matching failed: .*$", text, re.MULTILINE)
    if failure is not None:
        out["note"] = failure.group(0).strip()
    elif re.search(r"Traceback \(most recent call last\)", text):
        last = [line for line in text.splitlines() if line.strip()][-1]
        out["note"] = last.strip()[:160]
    return out


def gds_cells(archive: Path) -> dict:
    """The flow's PLM result from the routed GDS's kfactory meta info."""
    gds = next(archive.glob("routed_*.gds"), None)
    if gds is None:
        return {}
    try:
        import klayout.db as kdb
    except ImportError:
        return {}
    layout = kdb.Layout()
    layout.read(str(gds))
    top = next(cell for cell in layout.each_cell() if cell.is_top())
    info = next((meta.value for meta in top.each_meta_info() if meta.name == "kfactory:info"), None)
    if not isinstance(info, dict):
        return {}
    out: dict = {}
    analysis = info.get("path_length_analysis")
    if isinstance(analysis, dict):
        groups = [
            g for g in analysis.get("matching_group_diagnostics", []) or [] if isinstance(g, dict)
        ]
        outputs = [
            g for g in analysis.get("output_matching_diagnostics", []) or [] if isinstance(g, dict)
        ]
        out["plm_groups_gds"] = len(groups)
        out["group_spread_um"] = round(
            max((float(g.get("realized_arrival_spread_um", 0.0)) for g in groups), default=0.0), 6
        )
        out["output_spread_um"] = round(
            max((float(g.get("realized_arrival_spread_um", 0.0)) for g in outputs), default=0.0), 6
        )
        out["plm_over_tolerance_gds"] = sum(1 for g in groups if g.get("within_tolerance") is False)
        acceptance = analysis.get("path_length_acceptance")
        if isinstance(acceptance, dict):
            out["plm_accepted"] = bool(acceptance.get("passed"))
            out["max_physical_residual_um"] = round(
                float(acceptance.get("max_physical_residual_um", 0.0)), 6
            )
    meander = info.get("meander_insertion_report")
    if isinstance(meander, dict) and meander.get("total_inserted_extra_length_um") is not None:
        out["meander_inserted_um"] = round(float(meander["total_inserted_extra_length_um"]), 3)
    electrical = info.get("electrical_routing")
    if isinstance(electrical, dict) and electrical.get("verification_error_count") is not None:
        out["electrical_error_count"] = int(electrical["verification_error_count"])
    return out


def cells(run: Path) -> dict:
    """Every column of one archived run, in the baseline's JSON form."""
    m = read(run)
    log = log_cells(run / "run.log")
    gds = gds_cells(run)
    txt = (run / "run.txt").read_text() if (run / "run.txt").exists() else ""
    commit = re.search(r"commit=(\w+)", txt)
    rc = m.get("rc")
    out = {
        "rc": int(rc) if isinstance(rc, str) and rc.isdigit() else None,
        "photonic_error_count": m.get("photonic_error_count"),
        "electrical_error_count": gds.get("electrical_error_count"),
        "nets": m.get("routed_record_count", log.get("nets")),
        "gds_length_um": m.get("gds_length_um"),
        "meander_inserted_um": gds.get("meander_inserted_um", log.get("meander_inserted_um")),
        "plm_groups": log.get("plm_groups", gds.get("plm_groups_gds")),
        "plm_over_tolerance": log.get("plm_over_tolerance", gds.get("plm_over_tolerance_gds")),
        "group_spread_um": gds.get("group_spread_um"),
        "output_spread_um": gds.get("output_spread_um"),
        "plm_accepted": gds.get("plm_accepted"),
        "routing_loop_s": m.get("net_routing_phase_s"),
        "plm_phase_s": log.get("plm_phase_s"),
        "wall_s": m.get("wall_s"),
        "stamp": run.name,
        "commit": commit.group(1) if commit else None,
    }
    if log.get("note"):
        out["note"] = log["note"]
    if log.get("electrical_ok") not in (None, "yes") and out["electrical_error_count"] is None:
        out["note"] = (out.get("note", "") + f" electrical_ok={log['electrical_ok']}").strip()
    return out


def rows_of(root: Path, config: str) -> dict[str, dict]:
    runs = {b: run for b, c, run in latest_runs(root) if c == config}
    order = sorted(runs, key=lambda b: (ORDER.index(b) if b in ORDER else 99, b))
    return {b: cells(runs[b]) for b in order}


def fmt(value: object, digits: int | None = None) -> str:
    if value is None:
        return "--"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float) and digits is not None:
        return f"{value:.{digits}f}"
    return str(value)


def print_table(rows: dict[str, dict]) -> None:
    print(
        "| benchmark | rc | errors ph/el | nets | GDS length um | meander um | groups | over tol | "
        "group spread um | output spread um | t_loop s | t_plm s | wall s | note |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for bench, c in rows.items():
        print(
            f"| {bench} | {fmt(c['rc'])} | {fmt(c['photonic_error_count'])}/{fmt(c['electrical_error_count'])} | "
            f"{fmt(c['nets'])} | {fmt(c['gds_length_um'], 3)} | {fmt(c['meander_inserted_um'], 3)} | "
            f"{fmt(c['plm_groups'])} | {fmt(c['plm_over_tolerance'])} | {fmt(c['group_spread_um'], 4)} | "
            f"{fmt(c['output_spread_um'], 4)} | {fmt(c['routing_loop_s'], 1)} | {fmt(c['plm_phase_s'], 1)} | "
            f"{fmt(c['wall_s'])} | {c.get('note', '')} |"
        )


def mismatches(ours: dict, base: dict) -> list[str]:
    bad = []
    for key in EXACT_KEYS:
        if ours.get(key) != base.get(key):
            bad.append(f"{key} {fmt(base.get(key))}->{fmt(ours.get(key))}")
    for key in LENGTH_KEYS:
        a, b = ours.get(key), base.get(key)
        if (a is None) != (b is None) or (a is not None and abs(float(a) - float(b)) > TOL_UM):
            bad.append(f"{key} {fmt(b, 3)}->{fmt(a, 3)}")
    return bad


def compare(rows: dict[str, dict], baseline: dict[str, dict], only_present: bool) -> int:
    print("| benchmark | status |")
    print("|---|---|")
    bad = 0
    for bench in sorted(baseline, key=lambda b: (ORDER.index(b) if b in ORDER else 99, b)):
        ours = rows.get(bench)
        if ours is None:
            if only_present:
                continue
            print(f"| {bench} | MISSING |")
            bad += 1
            continue
        if ours["rc"] is None:
            print(f"| {bench} | RUNNING ({ours['stamp']}) |")
            bad += 1
            continue
        problems = mismatches(ours, baseline[bench])
        bad += bool(problems)
        print(f"| {bench} | {'ok' if not problems else 'MISMATCH ' + '; '.join(problems)} |")
    for bench in rows:
        if bench not in baseline:
            print(f"| {bench} | not in baseline (ignored) |")
    print(f"\n{bad} PLM cell(s) missing or mismatched" if bad else "\nall PLM cells match")
    return 1 if bad else 0


def main(argv: list[str]) -> int:
    if not argv or argv[0].startswith("--"):
        print(__doc__)
        return 2
    root = Path(argv[0])
    options = argv[1:]

    def option(name: str) -> str | None:
        return options[options.index(name) + 1] if name in options else None

    config = option("--config") or DEFAULT_CONFIG
    rows = rows_of(root, config)
    print_table(rows)
    json_path = option("--json")
    if json_path is not None:
        Path(json_path).write_text(json.dumps(rows, indent=2, sort_keys=False) + "\n")
        print(f"\nwrote {json_path} ({len(rows)} benchmark(s))")
    baseline_path = option("--compare")
    if baseline_path is not None:
        baseline = json.loads(Path(baseline_path).read_text())
        print()
        return compare(rows, baseline, "--only-present" in options)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
