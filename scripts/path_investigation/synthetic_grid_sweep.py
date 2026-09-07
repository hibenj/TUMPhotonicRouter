"""Generality sweep of contribution 2 on synthetic permutation layers.

Runs `routing_flow.py permutation_synthetic --preplaced-crossing-grids true`
(and the lidar-pure baseline as a control) over a grid of configurations and
tabulates: structure chosen per layer, derivation rejections, router
failures, verification errors, tiles vs planned crossings, and times.

    PYTHONPATH=. .venv/bin/python scripts/path_investigation/synthetic_grid_sweep.py [part]
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time

PY = ".venv/bin/python"


def configs(part: str) -> list[dict]:
    out = []
    if part in ("a", "all"):
        for k in (2, 4, 8):
            for tp in (50.0, 100.0):
                for band in (110.0, 300.0):
                    for mode, seed in (("random", 1), ("random", 2), ("swap", 1), ("reverse", 1)):
                        out.append(
                            {
                                "k": k,
                                "m": 2,
                                "sp": 5.0,
                                "tp": tp,
                                "band": band,
                                "mode": mode,
                                "seed": seed,
                            }
                        )
    if part in ("b", "all"):
        for tp in (50.0, 100.0):
            for band in (110.0, 300.0, 550.0):
                for mode, seed in (("random", 1), ("random", 3), ("swap", 1)):
                    out.append(
                        {
                            "k": 1,
                            "m": 8,
                            "sp": 0.0,
                            "tp": tp,
                            "band": band,
                            "mode": mode,
                            "seed": seed,
                        }
                    )
        for band in (110.0, 300.0):
            out.append(
                {"k": 4, "m": 4, "sp": 5.0, "tp": 100.0, "band": band, "mode": "random", "seed": 4}
            )
            out.append(
                {"k": 8, "m": 2, "sp": 22.0, "tp": 100.0, "band": band, "mode": "random", "seed": 5}
            )
    return out


def run(cfg: dict, grid: bool, timeout: float) -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = "."
    env["PHOTONIC_ROUTER_SYNTH"] = ",".join(f"{k}={v}" for k, v in cfg.items())
    args = [PY, "routing_flow.py", "permutation_synthetic"]
    if grid:
        args += ["--preplaced-crossing-grids", "true"]
    t0 = time.time()
    try:
        proc = subprocess.run(args, env=env, capture_output=True, text=True, timeout=timeout)
        out = proc.stdout + proc.stderr
        rc = proc.returncode
    except subprocess.TimeoutExpired as exc:
        out = (exc.stdout or "") + (exc.stderr or "") if isinstance(exc.stdout, str) else ""
        rc = "timeout"
    dt = time.time() - t0
    res = {"rc": rc, "t": round(dt, 1)}
    m = re.search(r"attempts=(\d+), failures=(\d+), simple=[\d/]+, repairs=(\d+)", out)
    if m:
        res["att"], res["fail"], res["rep"] = (int(m.group(i)) for i in (1, 2, 3))
    if grid:
        res["column_layers"] = len(re.findall(r"column grid stage", out))
        m = re.search(
            r"Grids: (\d+) placed, (\d+) crossing component\(s\) \(topology expects (\d+)\)", out
        )
        if m:
            res["tiles"], res["planned"] = int(m.group(2)), int(m.group(3))
            res["structure"] = (
                "column" if res["column_layers"] else ("X" if res["tiles"] else "none")
            )
        m = re.search(r"ValueError: (column grid:[^\n]*|stretched[^\n]*|[^\n]*lattice[^\n]*)", out)
        if m:
            res["rejected"] = m.group(1)[:90]
    m = re.search(r"RuntimeError: No route found for ([^:]+)", out)
    if m:
        res["route_fail"] = m.group(1)
    if "Photonic geometry verification failed" in out:
        res["verify_fail"] = True
    try:
        with open("build/verification/permutation_synthetic_photonic_verification.json") as handle:
            res["ver_errors"] = json.load(handle).get("error_count")
    except Exception:
        pass
    return res


def main() -> None:
    part = sys.argv[1] if len(sys.argv) > 1 else "all"
    start = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    stop = int(sys.argv[3]) if len(sys.argv) > 3 else 10**6
    rows = []
    for cfg in configs(part)[start:stop]:
        g = run(cfg, True, 90)
        c = (
            run(cfg, False, 30)
            if os.environ.get("SWEEP_CONTROL", "1") != "0"
            else {"rc": "skipped", "t": 0.0}
        )
        seed = cfg["seed"] if cfg["mode"] == "random" else ""
        label = (
            f"k={cfg['k']} m={cfg['m']} sp={cfg['sp']:g} tp={cfg['tp']:g} "
            f"band={cfg['band']:g} {cfg['mode']}{seed}"
        )
        g_state = (
            "rejected: " + g["rejected"]
            if "rejected" in g
            else "route fail " + g["route_fail"]
            if "route_fail" in g
            else "verify fail"
            if g.get("verify_fail")
            else ("clean" if g.get("rc") == 0 else f"rc={g.get('rc')}")
        )
        c_state = (
            "clean"
            if c.get("rc") == 0
            else (
                "timeout"
                if c.get("rc") == "timeout"
                else ("route fail" if "route_fail" in c else f"rc={c.get('rc')}")
            )
        )
        rows.append(
            (
                label,
                g.get("structure", "-"),
                g.get("planned", "-"),
                g.get("tiles", "-"),
                f"{g.get('att', '-')}/{g.get('fail', '-')}/{g.get('rep', '-')}",
                g_state,
                g["t"],
                c_state,
                f"{c.get('att', '-')}/{c.get('fail', '-')}/{c.get('rep', '-')}",
                c["t"],
            )
        )
        print("| " + " | ".join(str(x) for x in rows[-1]) + " |", flush=True)
    print()
    print(
        "| config | structure | planned | tiles | grid att/fail/rep | grid result | grid s "
        "| lidar-pure result | lp att/fail/rep | lp s |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print("| " + " | ".join(str(x) for x in r) + " |")


if __name__ == "__main__":
    main()
