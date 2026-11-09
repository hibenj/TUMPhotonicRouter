"""Anatomy of the negotiated loop's search attempts from archived run logs.

Reads the `native_negotiated_search` trace lines (runs with
`PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1`, as `scripts/results/reproduce_paper.sh`
archives them) of every cell under a results root and prints, per cell and
in total: the attempts by kind, budget and outcome with their seconds and
expansions, the share of search time spent on attempts that failed, the
expansion distribution of the successful first attempts and retries, and
what the retry did after a failed first attempt.

Usage: attempt_anatomy.py <results_root> [--cells all|heavy] [--markdown]
"""

from __future__ import annotations

import argparse
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path

LINE = re.compile(
    r"native_negotiated_search net=(\d+) kind=(\w+) elapsed_s=([0-9.]+) "
    r"expanded=(\S+) outcome=(\w+) budget=(\S+)"
)
NET = re.compile(r"native_negotiated_net index=\d+ round=(\d+) net=(\d+) failed_count=(\d+)")
HEAVY = {
    ("benes_32x32_flat", "lidar-pure"),
    ("benes_32x32_flat", "contribution1"),
    ("benes_128x128_flat", "contribution2"),
    ("multiportmmi_32x32", "lidar-pure"),
    ("multiportmmi_64x64", "lidar-pure"),
    ("multiportmmi_64x64", "contribution1"),
    ("multiportmmi_128x128", "contribution2"),
}


def q(xs: list[float], p: float) -> float | None:
    if not xs:
        return None
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))]


def fmt_m(v: float | None) -> str:
    return "-" if v is None else f"{v / 1e6:.2f} M"


def parse(log: Path) -> list[dict]:
    attempts: list[dict] = []
    round_of_net: dict[str, int] = {}
    for line in log.read_text(errors="replace").splitlines():
        m = NET.search(line)
        if m:
            round_of_net[m.group(2)] = int(m.group(1))
            continue
        m = LINE.search(line)
        if not m:
            continue
        net, kind, secs, expanded, outcome, budget = m.groups()
        attempts.append(
            {
                "net": net,
                "round": round_of_net.get(net, 0),
                "kind": kind,
                "s": float(secs),
                "expanded": int(expanded) if expanded.isdigit() else None,
                "outcome": outcome,
                "budget": budget,
            }
        )
    return attempts


def summarize(attempts: list[dict]) -> dict:
    by_key: dict[tuple[str, str, str], list[dict]] = defaultdict(list)
    for a in attempts:
        by_key[(a["kind"], a["budget"], a["outcome"])].append(a)
    total_s = sum(a["s"] for a in attempts)
    failed_s = sum(a["s"] for a in attempts if a["outcome"] != "routed")
    # retry outcome after a failed plain first attempt (same net, same round)
    plain_failed = {
        (a["net"], a["round"])
        for a in attempts
        if a["kind"] == "plain" and a["outcome"] != "routed"
    }
    retry = Counter()
    for a in attempts:
        if a["kind"] == "plain_retry" and (a["net"], a["round"]) in plain_failed:
            retry[a["outcome"]] += 1
    ok_first = [
        a["expanded"]
        for a in attempts
        if a["kind"] == "plain" and a["outcome"] == "routed" and a["expanded"]
    ]
    ok_retry = [
        a["expanded"]
        for a in attempts
        if a["kind"] == "plain_retry" and a["outcome"] == "routed" and a["expanded"]
    ]
    return {
        "rows": by_key,
        "total_s": total_s,
        "failed_s": failed_s,
        "retry_after_fail": retry,
        "ok_first": ok_first,
        "ok_retry": ok_retry,
        "n": len(attempts),
    }


def print_cell(name: str, s: dict) -> None:
    print(
        f"\n### {name}: {s['n']} attempts, {s['total_s']:.0f} s search, failed attempts {s['failed_s']:.0f} s "
        f"({100 * s['failed_s'] / max(s['total_s'], 1e-9):.0f} %)"
    )
    print("| kind | budget | outcome | n | seconds | expansions |")
    print("|---|---|---|---|---|---|")
    for (kind, budget, outcome), rows in sorted(
        s["rows"].items(), key=lambda kv: -sum(r["s"] for r in kv[1])
    ):
        secs = sum(r["s"] for r in rows)
        exp = sum(r["expanded"] or 0 for r in rows)
        print(f"| {kind} | {budget} | {outcome} | {len(rows)} | {secs:.0f} | {fmt_m(exp)} |")
    if s["retry_after_fail"]:
        print(f"retry after a failed first attempt: {dict(s['retry_after_fail'])}")
    if s["ok_first"]:
        o = s["ok_first"]
        print(
            f"successful first attempts: n={len(o)} median {fmt_m(statistics.median(o))}, p90 {fmt_m(q(o, 0.9))}, "
            f"max {fmt_m(max(o))}, above 1 M: {sum(1 for x in o if x > 1e6)}, above 0.5 M: {sum(1 for x in o if x > 5e5)}"
        )
    if s["ok_retry"]:
        o = s["ok_retry"]
        print(
            f"successful retries: n={len(o)} median {fmt_m(statistics.median(o))}, p90 {fmt_m(q(o, 0.9))}, "
            f"max {fmt_m(max(o))}, above 2 M: {sum(1 for x in o if x > 2e6)}, above 5 M: {sum(1 for x in o if x > 5e6)}"
        )


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("root", type=Path)
    ap.add_argument("--cells", default="heavy", choices=["all", "heavy"])
    args = ap.parse_args()
    all_attempts: list[dict] = []
    for log in sorted(args.root.glob("*/*/*/run.log")):
        bench, cfg = log.parts[-4], log.parts[-3]
        attempts = parse(log)
        all_attempts.extend(attempts)
        if args.cells == "all" or (bench, cfg) in HEAVY:
            print_cell(f"{bench} {cfg}", summarize(attempts))
    print_cell("ALL CELLS", summarize(all_attempts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
