"""Aggregate the `exp_f_profile` lines of a run log.

Each line is one successful kernel search (patch
`patches/0001-kernel-f-value-profile.patch`): the number of pops, how many
of them had an f-score below the final cost C* (strictly, and below 99.9 /
99 / 95 / 90 % of it), the ratio h(start) / C* and C*. The summary groups
the searches by size and reports the pop-weighted shares, which say how much
of the kernel's work any admissible heuristic has to do (pops below C*) and
how much is the plateau at f = C* (tie-breaking), and how far the current
heuristic sits below the optimum at the start.

Usage: f_profile_summary.py <run.log or run.txt> ...
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

PAIR = re.compile(r"(\w+)=(-?[0-9.]+)")
FIELDS = (
    "pops",
    "below_cstar",
    "below_0999",
    "below_099",
    "below_095",
    "below_090",
    "h_start_ratio",
    "h2d_len_ratio",
    "h2d_priced_ratio",
    "cstar",
    "length",
    "bend_cost",
    "other",
    "h2d_walls_ratio",
)
GROUPS = [
    (10_000, "up to 10 k pops"),
    (100_000, "10 k to 100 k"),
    (1_000_000, "100 k to 1 M"),
    (10**12, "above 1 M"),
]


def ratios(g: list, pops: float) -> str:
    """Pop-weighted mean and minimum of h(start)/C* and the two 2D bounds."""
    out = ""
    for i in (6, 7, 8):
        vals = [r for r in g if r[i] == r[i]]  # drop NaN (lines without the bound)
        if not vals:
            out += " | -"
            continue
        w = sum(r[0] for r in vals)
        out += f" | {sum(r[i] * r[0] for r in vals) / w:.3f}, {min(r[i] for r in vals):.3f}"
    vals = [r for r in g if r[10] == r[10]]
    if vals:
        w = sum(r[0] for r in vals)
        shares = [sum(r[i] / r[9] * r[0] for r in vals) / w for i in (10, 11, 12)]
        out += " | " + " + ".join(f"{100 * x:.0f} %" for x in shares)
    else:
        out += " | -"
    return out + " |"


def main() -> None:
    rows = []
    for path in sys.argv[1:]:
        for line in Path(path).read_text(errors="replace").splitlines():
            if "exp_f_profile " not in line:
                continue
            kv = dict(PAIR.findall(line.split("exp_f_profile ", 1)[1]))
            rows.append(tuple(float(kv[f]) if f in kv else float("nan") for f in FIELDS))
    print(f"{len(rows)} successful searches with a profile")
    print(
        "| searches | n | pops | below C* | below 0.999 C* | below 0.99 C* | below 0.95 C* | below 0.90 C* "
        "| h(start)/C* (pop-weighted mean, min) | 2D length bound / C* | 2D priced bound / C* "
        "| C* = length + bends + crossings (pop-weighted shares) |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    lower = 0
    for limit, name in GROUPS:
        g = [r for r in rows if lower < r[0] <= limit]
        lower = limit
        if not g:
            continue
        pops = sum(r[0] for r in g)
        shares = [100 * sum(r[i] for r in g) / pops for i in range(1, 6)]
        print(
            f"| {name} | {len(g)} | {pops / 1e6:.2f} M | "
            + " | ".join(f"{s:.1f} %" for s in shares)
            + ratios(g, pops)
        )
    pops = sum(r[0] for r in rows)
    if pops:
        shares = [100 * sum(r[i] for r in rows) / pops for i in range(1, 6)]
        print(
            f"| all | {len(rows)} | {pops / 1e6:.2f} M | "
            + " | ".join(f"{s:.1f} %" for s in shares)
            + ratios(rows, pops)
        )


if __name__ == "__main__":
    main()
