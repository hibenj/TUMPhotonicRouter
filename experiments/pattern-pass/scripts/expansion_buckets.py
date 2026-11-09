"""Search time by expansion count, from the trace lines of an archived table.

A pattern pass in front of the kernel (research item 8) can only replace the
searches whose route a pattern would find: short searches. This script
buckets every `native_negotiated_search` attempt of a results root by its
expansion count and sums the elapsed seconds per bucket, so the ceiling of
what a pattern pass could save is read off the small buckets. Failed
attempts report `expanded=0` in the trace and land in the first bucket; the
table shows them separately.

Usage: expansion_buckets.py <results_root> [cell ...]   (cell as benchmark/config)
"""

from __future__ import annotations

import re
import sys
from collections import defaultdict
from pathlib import Path

LINE = re.compile(
    r"native_negotiated_search net=(\d+) kind=(\w+) elapsed_s=([0-9.]+) "
    r"expanded=(\S+) outcome=(\w+) budget=(\S+)"
)
BUCKETS = [
    (0, "0 (simple route, or a failed attempt)"),
    (1_000, "<= 1 k"),
    (10_000, "<= 10 k"),
    (100_000, "<= 100 k"),
    (1_000_000, "<= 1 M"),
    (10**12, "> 1 M"),
]


def bucket(expanded: int) -> str:
    for limit, name in BUCKETS:
        if expanded <= limit:
            return name
    raise AssertionError


def show(name: str, d: dict) -> None:
    total = sum(v[1] for v in d.values()) or 1e-9
    print(f"\n### {name}: {total:.0f} s of search")
    print("| expansions | outcome | attempts | seconds | share of search time |")
    print("|---|---|---|---|---|")
    for _, bucket_name in BUCKETS:
        for outcome in ("routed", "failed"):
            v = d.get((bucket_name, outcome))
            if v:
                print(
                    f"| {bucket_name} | {outcome} | {v[0]} | {v[1]:.1f} | {100 * v[1] / total:.1f} % |"
                )


def main() -> None:
    root = Path(sys.argv[1])
    wanted = {tuple(c.split("/")) for c in sys.argv[2:]}
    total: dict = defaultdict(lambda: [0, 0.0])
    cells: dict = {}
    for log in sorted(root.glob("*/*/*/run.log")):
        cell = (log.parts[-4], log.parts[-3])
        per_cell: dict = defaultdict(lambda: [0, 0.0])
        for m in LINE.finditer(log.read_text(errors="replace")):
            key = (bucket(int(m[4])), "routed" if m[5] == "routed" else "failed")
            for d in (per_cell, total):
                d[key][0] += 1
                d[key][1] += float(m[3])
        cells[cell] = per_cell
    show(f"ALL CELLS ({root.name})", total)
    for cell, d in cells.items():
        if not wanted or cell in wanted:
            show(" ".join(cell), d)


if __name__ == "__main__":
    main()
