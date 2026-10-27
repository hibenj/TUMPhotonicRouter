"""Ablation table over results_ablation/<variant>/ (scripts/results/ablation.sh).

One row per cell (benchmark, configuration); per variant the crossings, the
GDS length and the routing-loop time, and against the reference variant the
length and time ratios. The last rows are the geometric means of the ratios
over the cells every variant completed, plus the crossing total.

Usage: ablation_table.py [results_ablation] [--reference reference]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from table import ORDER, latest_runs, read


def cells(root: Path) -> dict[tuple[str, str], dict]:
    out: dict[tuple[str, str], dict] = {}
    if not root.is_dir():
        return out
    for bench, cfg, run in latest_runs(root):
        m = read(run)
        if m.get("rc") != "0":
            continue
        out[(bench, cfg)] = {
            "x": m.get("crossing_count"),
            "len": m.get("gds_length_um"),
            "t": m.get("routing_loop_s", m.get("net_routing_phase_s")),
        }
    return out


def fmt(v: object, digits: int = 3) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", nargs="?", default="results_ablation")
    parser.add_argument("--reference", default="reference")
    args = parser.parse_args()
    root = Path(args.root)
    variants = sorted(d.name for d in root.iterdir() if d.is_dir()) if root.is_dir() else []
    if args.reference in variants:
        variants.remove(args.reference)
        variants.insert(0, args.reference)
    data = {v: cells(root / v) for v in variants}
    ref = data.get(args.reference, {})
    keys = sorted(
        {k for d in data.values() for k in d},
        key=lambda k: (
            ORDER.index(k[0].replace("_flat", "")) if k[0].replace("_flat", "") in ORDER else 99,
            k[1],
        ),
    )
    head = "| cell | " + " | ".join(f"{v} X / L um / t s" for v in variants) + " |"
    print(head)
    print("|" + "---|" * (len(variants) + 1))
    for k in keys:
        row = [f"{k[0]} {k[1]}"]
        for v in variants:
            c = data[v].get(k)
            if c is None:
                row.append("not completed")
                continue
            text = f"{fmt(c['x'])} / {fmt(c['len'], 1)} / {fmt(c['t'], 1)}"
            r = ref.get(k)
            if (
                v != args.reference
                and r
                and r.get("len")
                and r.get("t")
                and c.get("len")
                and c.get("t")
            ):
                text += f" ({c['len'] / r['len']:.4f}x L, {c['t'] / r['t']:.2f}x t"
                if c["x"] is not None and r["x"] is not None and c["x"] != r["x"]:
                    text += f", {c['x'] - r['x']:+d} X"
                text += ")"
            row.append(text)
        print("| " + " | ".join(row) + " |")
    # geometric means over the cells every variant completed
    common = [
        k
        for k in keys
        if all(k in data[v] and data[v][k].get("len") and data[v][k].get("t") for v in variants)
    ]
    print()
    print(
        f"Geometric means over {len(common)} cells completed by every variant (ratio to {args.reference}):"
    )
    print("| variant | length ratio | loop-time ratio | crossings total | cells completed |")
    print("|---|---|---|---|---|")
    for v in variants:
        if v == args.reference or not common:
            total = sum(c["x"] or 0 for c in data[v].values())
            print(f"| {v} | 1 | 1 | {total} | {len(data[v])} |")
            continue
        lr = math.exp(
            sum(math.log(data[v][k]["len"] / ref[k]["len"]) for k in common) / len(common)
        )
        tr = math.exp(sum(math.log(data[v][k]["t"] / ref[k]["t"]) for k in common) / len(common))
        total = sum(c["x"] or 0 for c in data[v].values())
        print(f"| {v} | {lr:.4f} | {tr:.3f} | {total} | {len(data[v])} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
