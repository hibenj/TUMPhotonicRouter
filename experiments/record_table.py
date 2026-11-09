"""Compact cell record of a 27-cell run for `experiments/<name>/tables/`.

Reads the `reproduce.txt` that `scripts/results/reproduce_paper.sh` writes
(one line per cell: config, benchmark, rc, wall seconds, run end) and the
`metrics.json` of each archived run, and writes the record that
`experiments/table.py` renders.

Usage: experiments/record_table.py <results_root> <out.json> --commit <sha> --machine <text> [--switch <text>]
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

LINE = re.compile(
    r"^(?P<ts>\S+) (?P<config>\S+) (?P<benchmark>\S+) \| rc=(?P<rc>-?\d+) wall=(?P<wall>\d+)s"
)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("results_root", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--commit", required=True)
    ap.add_argument("--machine", required=True)
    ap.add_argument("--switch", default="")
    args = ap.parse_args()

    cells = []
    for line in (args.results_root / "reproduce.txt").read_text().splitlines():
        m = LINE.match(line)
        if not m:
            continue
        cell_dir = args.results_root / m["benchmark"] / m["config"]
        runs = sorted(p for p in cell_dir.iterdir() if p.is_dir()) if cell_dir.is_dir() else []
        run = runs[-1] if runs else None
        metrics_path = run / "metrics.json" if run else None
        metrics = (
            json.loads(metrics_path.read_text())
            if metrics_path is not None and metrics_path.is_file()
            else {}
        )
        cells.append(
            {
                "benchmark": m["benchmark"],
                "config": m["config"],
                "run": run.name if run else None,
                "rc": int(m["rc"]),
                "wall_s": int(m["wall"]),
                "metrics": metrics,
            }
        )
    cells.sort(key=lambda c: (c["benchmark"], c["config"]))
    doc = {"commit": args.commit, "machine": args.machine, "switch": args.switch, "cells": cells}
    args.out.write_text(json.dumps(doc, indent=1, sort_keys=True) + "\n")
    print(f"{len(cells)} cells -> {args.out}")


if __name__ == "__main__":
    main()
