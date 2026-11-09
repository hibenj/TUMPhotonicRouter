"""Markdown table over the compact cell records of an experiment.

Each record file (`<experiment>/tables/<variant>.json`) holds the 27 paper
cells of one table run: per cell the benchmark, the configuration, the exit
code, the wall seconds and the `metrics.json` of the run. The first argument
is the reference; every further file is compared with it cell by cell on
crossings, GDS length, verifier errors and wall time. With `--pins` the
reference is the pinned table (`docs/paper_table_sources.json`, the `current`
block where one exists) instead of a record file.

Usage: experiments/table.py [--pins docs/paper_table_sources.json] <ref.json> <variant.json> ...
       experiments/table.py --pins docs/paper_table_sources.json <variant.json> ...

A variant cell that did not finish (rc != 0 or no length) prints `fail` and
is left out of the totals. The last rows are the number of cells identical
to the reference (crossings equal, length within 0.01 um), the geometric mean
of the length ratios and the total wall ratio over the cells every table
finished. Wall times come from a shared machine and compare only tables run
on the same one.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

ORDER = [
    "benes_4x4_flat",
    "benes_8x8_flat",
    "benes_16x16_flat",
    "benes_32x32_flat",
    "benes_64x64_flat",
    "benes_128x128_flat",
    "multiportmmi_8x8",
    "multiportmmi_16x16",
    "multiportmmi_32x32",
    "multiportmmi_64x64",
    "multiportmmi_128x128",
]
CONFIGS = ["lidar-pure", "contribution1", "contribution2"]
PIN_KEYS = {
    "baseline": "lidar-pure",
    "contribution1": "contribution1",
    "contribution2": "contribution2",
}

Cell = dict[str, object]


def load_records(path: Path) -> dict[tuple[str, str], Cell]:
    doc = json.loads(path.read_text())
    out: dict[tuple[str, str], Cell] = {}
    for c in doc["cells"]:
        m = c.get("metrics") or {}
        ok = c.get("rc") == 0 and m.get("gds_length_um") is not None
        out[(c["benchmark"], c["config"])] = {
            "ok": ok,
            "x": m.get("crossing_count"),
            "len": m.get("gds_length_um"),
            "err": m.get("photonic_error_count"),
            "rounds": m.get("negotiated_rounds"),
            "wall": c.get("wall_s"),
        }
    return out


def load_pins(path: Path) -> dict[tuple[str, str], Cell]:
    doc = json.loads(path.read_text())
    out: dict[tuple[str, str], Cell] = {}
    for rec in doc["sources"].values():
        bench = rec["ours_benchmark"]
        for key, cfg in PIN_KEYS.items():
            r = rec.get(key)
            if not r or "gds_length_um" not in r:
                continue
            cur = r.get("current") or {}
            out[(bench, cfg)] = {
                "ok": True,
                "x": cur.get("crossing_count", r["crossing_count"]),
                "len": cur.get("gds_length_um", r["gds_length_um"]),
                "err": 0,
                "rounds": None,
                "wall": None,
            }
    return out


def fmt(v: object, digits: int = 3) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--pins", type=Path, help="pinned table as the reference")
    ap.add_argument("files", nargs="+", type=Path)
    args = ap.parse_args()
    if args.pins:
        ref = load_pins(args.pins)
        ref_name = "pins"
        variants = [(f.stem, load_records(f)) for f in args.files]
    else:
        ref = load_records(args.files[0])
        ref_name = args.files[0].stem
        variants = [(f.stem, load_records(f)) for f in args.files[1:]]
    head = ["benchmark", "config", f"{ref_name} crossings / length um / wall s"]
    for name, _ in variants:
        head.append(f"{name} crossings / length / wall")
        head.append("length ratio / wall ratio")
    print("| " + " | ".join(head) + " |")
    print("|" + "---|" * len(head))
    same = [0] * len(variants)
    counted = [0] * len(variants)
    log_len = [0.0] * len(variants)
    wall_ref = [0.0] * len(variants)
    wall_var = [0.0] * len(variants)
    for bench in ORDER:
        for cfg in CONFIGS:
            key = (bench, cfg)
            r = ref.get(key)
            if r is None or not any(key in v for _, v in variants):
                continue
            row = [bench, cfg, f"{fmt(r['x'])} / {fmt(r['len'])} / {fmt(r['wall'])}"]
            for i, (_, v) in enumerate(variants):
                c = v.get(key)
                if c is None or not c["ok"]:
                    row += ["fail" if c is not None else "-", "-"]
                    continue
                err = "" if not c["err"] else f" ({c['err']} errors)"
                row.append(f"{fmt(c['x'])} / {fmt(c['len'])} / {fmt(c['wall'])}{err}")
                lr = (c["len"] / r["len"]) if r["len"] else None
                wr = (c["wall"] / r["wall"]) if (c["wall"] and r["wall"]) else None
                row.append(f"{fmt(lr, 4)} / {fmt(wr, 2)}")
                counted[i] += 1
                if c["x"] == r["x"] and abs(c["len"] - r["len"]) < 0.01:
                    same[i] += 1
                if lr:
                    log_len[i] += math.log(lr)
                if wr:
                    wall_ref[i] += r["wall"]
                    wall_var[i] += c["wall"]
            print("| " + " | ".join(row) + " |")
    foot = ["cells identical to the reference", "", ""]
    for i in range(len(variants)):
        foot += [f"{same[i]} of {counted[i]}", ""]
    print("| " + " | ".join(foot) + " |")
    foot = ["geometric mean length ratio / total wall ratio", "", ""]
    for i in range(len(variants)):
        gm = math.exp(log_len[i] / counted[i]) if counted[i] else None
        wr = (wall_var[i] / wall_ref[i]) if wall_ref[i] else None
        foot += ["", f"{fmt(gm, 4)} / {fmt(wr, 2)}"]
    print("| " + " | ".join(foot) + " |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
