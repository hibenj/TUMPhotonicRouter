"""Experiment 1, section 4 detail: per seed of the net-order runs, the
first-retry / probe / rip-up counts, the summed expansions of the trace lines
and the routing-loop seconds, next to the gate's default-order run.

Usage: order_detail.py <results_exp1 root> <gate root>"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path


def latest(d: Path) -> Path:
    return max(p for p in d.iterdir() if p.is_dir())


def describe(run: Path) -> str:
    text = (run / "run.log").read_text(errors="replace")
    att = [
        dict(re.findall(r"(\w+)=(\S+)", line))
        for line in text.splitlines()
        if "native_negotiated_search" in line
    ]
    retries = sum(1 for d in att if d["kind"] == "plain_retry")
    expanded = sum(int(d["expanded"]) for d in att if d["expanded"].isdigit())
    probes = len(re.findall("native_repair_probe net=", text))
    ripups = len(re.findall("native_repair_local_ripup|native_repair_global_ripup", text))
    metrics = json.loads((run / "metrics.json").read_text())
    extra = (f"+p{probes}" if probes else "") + (f"+rip{ripups}" if ripups else "")
    return f"{retries}{extra} / {expanded / 1e6:.2f} / {metrics.get('routing_loop_s')}"


def main() -> int:
    root = Path(sys.argv[1]) / "order"
    gate = Path(sys.argv[2])
    print("| cell | config | order | per seed: plain_retry(+probes+rip-ups) / expansions M / t s |")
    print("|---|---|---|---|")
    for bench in ("benes_4x4_flat", "benes_8x8_flat", "multiportmmi_8x8"):
        for cfg in ("lidar-pure", "contribution1", "contribution2"):
            print(f"| {bench} | {cfg} | default (gate) | {describe(latest(gate / bench / cfg))} |")
            for policy in ("random", "topological-random"):
                cells = [
                    f"s{seed}: {describe(latest(root / policy / f'seed{seed}' / bench / cfg))}"
                    for seed in range(1, 6)
                ]
                print(f"| {bench} | {cfg} | {policy} | {'; '.join(cells)} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
