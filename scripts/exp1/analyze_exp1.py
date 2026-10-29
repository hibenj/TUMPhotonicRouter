"""Experiment 1 analysis (2026-10-29): reads the archives scripts/exp1/run_exp1.sh
wrote under results_exp1/ and prints the report tables as Markdown.

Usage: analyze_exp1.py <results_exp1 root> <docs/date2027_table_sources.json> [<gate root>]

Sections: (1) cost per expansion and crossing-hook share from the `EXP1
run-summary` / `EXP1 search` stderr lines (PHOTONIC_ROUTER_EXP_KERNEL_PROFILE=1),
(2) heuristic quality per successful kernel search (final cost / h(start),
GDS length / Euclid, expansions per route cell), (3) attempt structure from the
`native_negotiated_search` trace lines (PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1),
(4) net-order sensitivity: `--net-order random` / `topological-random` under
five seeds against the paper's archived values and the gate's values on this
machine (<gate root>, the directory GATE_KEEP=1 kept)."""

from __future__ import annotations

import json
import re
import statistics
import sys
from collections import Counter
from pathlib import Path

BENCHES = ("benes_4x4_flat", "benes_8x8_flat", "multiportmmi_8x8", "benes_16x16_flat")
CONFIGS = ("lidar-pure", "contribution1", "contribution2")
PAPER_KEY = {
    "lidar-pure": "baseline",
    "contribution1": "contribution1",
    "contribution2": "contribution2",
}


def latest(root: Path, bench: str, config: str) -> Path | None:
    d = root / bench / config
    if not d.is_dir():
        return None
    stamps = sorted(p for p in d.iterdir() if p.is_dir())
    return stamps[-1] if stamps else None


def kv_line(line: str) -> dict[str, str]:
    return dict(re.findall(r"(\w+)=(\S+)", line))


def pct(values, q):
    if not values:
        return float("nan")
    s = sorted(values)
    k = max(0, min(len(s) - 1, round(q * (len(s) - 1))))
    return s[k]


def fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float):
        if v != v:
            return "nan"
        return f"{v:.{nd}f}"
    return str(v)


def section_profile(root: Path) -> None:
    print("## 1. Cost per expansion and crossing-hook share\n")
    print(
        "Source: `EXP1 run-summary` line per run (kernel searches only; simple routes never enter the kernel). "
        "`us/exp` = kernel-loop wall time / expanded states; `us/exp no hook` = (loop - hook time) / expanded; "
        "`hook ns/call` = hook time / hook calls; `hook/gen` = hook calls per generated neighbour.\n"
    )
    print(
        "| cell | config | kernel searches (with hook) | expanded | generated | loop ms | us/exp | hook calls | hook ms | hook share | us/exp no hook | hook ns/call | hook/gen | gen/exp |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for bench in BENCHES:
        for config in CONFIGS:
            run = latest(root / "profile", bench, config)
            if run is None:
                continue
            text = (run / "run.log").read_text(errors="replace")
            lines = [
                kv_line(line) for line in text.splitlines() if line.startswith("EXP1 run-summary")
            ]
            if not lines:
                print(f"| {bench} | {config} | (no run-summary line) |")
                continue
            # several batches possible: sum
            tot = Counter()
            for d in lines:
                for k in ("searches", "crossing_searches", "expanded", "generated", "hook_calls"):
                    tot[k] += int(d[k])
                tot["loop_ms"] += float(d["loop_ms"])
                tot["hook_ms"] += float(d["hook_ms"])
            exp = tot["expanded"] or 1
            us_exp = tot["loop_ms"] * 1000 / exp
            us_exp_nohook = (tot["loop_ms"] - tot["hook_ms"]) * 1000 / exp
            hook_share = tot["hook_ms"] / tot["loop_ms"] if tot["loop_ms"] else float("nan")
            ns_call = (
                tot["hook_ms"] * 1e6 / tot["hook_calls"] if tot["hook_calls"] else float("nan")
            )
            print(
                f"| {bench} | {config} | {tot['searches']} ({tot['crossing_searches']}) | {tot['expanded']} | {tot['generated']} | {tot['loop_ms']:.0f} | {us_exp:.2f} | {tot['hook_calls']} | {tot['hook_ms']:.1f} | {hook_share:.3f} | {us_exp_nohook:.2f} | {ns_call:.0f} | {tot['hook_calls'] / max(1, tot['generated']):.3f} | {tot['generated'] / exp:.2f} |"
            )
    print()


def search_records(run: Path) -> list[dict]:
    text = (run / "run.log").read_text(errors="replace")
    out = []
    for line in text.splitlines():
        if line.startswith("EXP1 search"):
            d = kv_line(line)
            out.append(
                {
                    k: (float(v) if re.fullmatch(r"-?[0-9.]+(e[-+]?\d+)?|nan|NaN", v) else v)
                    for k, v in d.items()
                }
            )
    return out


def section_heuristic(root: Path) -> None:
    print("## 2. Heuristic quality\n")
    print(
        "Source: `EXP1 search` lines of the successful, non-probe (`ignore_dyn=0`) kernel searches of the profile runs; "
        "`cost/h0` = final route cost / admissible estimate at the source (the kernel's own `SearchHeuristic`, "
        "Euclid or octile + minimum bend); `len/euclid` = route length / Euclidean grid distance (um); "
        "`exp/cell` = expanded states / route cells. Probe searches (`ignore_dyn=1`) and failed searches are counted separately.\n"
    )
    print(
        "| cell | config | ok searches | probes | failed | cost/h0 med / p90 / max | len/euclid med / p90 / max | share len/euclid > 1.5 | exp/cell med / p90 / max | expansions on failed searches |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|")
    for bench in BENCHES:
        for config in CONFIGS:
            run = latest(root / "profile", bench, config)
            if run is None:
                continue
            recs = search_records(run)
            ok = [r for r in recs if r["ok"] == 1 and r["ignore_dyn"] == 0 and r["simple"] == 0]
            probes = [r for r in recs if r["ignore_dyn"] == 1]
            failed = [r for r in recs if r["ok"] == 0]
            ch = [r["cost"] / r["h0"] for r in ok if r["h0"] > 0]
            le = [r["len_um"] / r["euclid"] for r in ok if r["euclid"] > 0]
            ec = [r["expanded"] / r["cells"] for r in ok if r["cells"] > 0]
            share = sum(1 for v in le if v > 1.5) / len(le) if le else float("nan")
            fexp = sum(int(r["expanded"]) for r in failed)
            texp = sum(int(r["expanded"]) for r in recs) or 1
            print(
                f"| {bench} | {config} | {len(ok)} | {len(probes)} | {len(failed)} | "
                f"{fmt(pct(ch, 0.5))} / {fmt(pct(ch, 0.9))} / {fmt(max(ch) if ch else None)} | "
                f"{fmt(pct(le, 0.5))} / {fmt(pct(le, 0.9))} / {fmt(max(le) if le else None)} | {share:.2f} | "
                f"{fmt(pct(ec, 0.5), 0)} / {fmt(pct(ec, 0.9), 0)} / {fmt(max(ec) if ec else None, 0)} | {fexp} ({100 * fexp / texp:.0f} %) |"
            )
    print()
    print(
        "Heavy searches only (successful, non-probe, >= 10000 expansions: the ones a better heuristic would speed up), with their share of the run's expansions:\n"
    )
    print(
        "| cell | config | heavy searches | share of expansions | cost/h0 med / max | len/euclid med / max | exp/cell med / max | h0 um min / max |"
    )
    print("|---|---|---|---|---|---|---|---|")
    for bench in BENCHES:
        for config in CONFIGS:
            run = latest(root / "profile", bench, config)
            if run is None:
                continue
            recs = search_records(run)
            heavy = [
                r for r in recs if r["ok"] == 1 and r["ignore_dyn"] == 0 and r["expanded"] >= 10000
            ]
            texp = sum(r["expanded"] for r in recs) or 1
            ch = [r["cost"] / r["h0"] for r in heavy if r["h0"] > 0]
            le = [r["len_um"] / r["euclid"] for r in heavy if r["euclid"] > 0]
            ec = [r["expanded"] / r["cells"] for r in heavy if r["cells"] > 0]
            h0s = [r["h0"] for r in heavy]
            print(
                f"| {bench} | {config} | {len(heavy)} | {sum(r['expanded'] for r in heavy) / texp:.2f} | "
                f"{fmt(pct(ch, 0.5))} / {fmt(max(ch) if ch else None)} | {fmt(pct(le, 0.5))} / {fmt(max(le) if le else None)} | "
                f"{fmt(pct(ec, 0.5), 0)} / {fmt(max(ec) if ec else None, 0)} | {fmt(min(h0s) if h0s else None, 0)} / {fmt(max(h0s) if h0s else None, 0)} |"
            )
    print()
    print(
        "Expansions per search, successful non-probe kernel searches (median / p90 / max) and the share of all expansions spent by the top 10 % of searches:\n"
    )
    print(
        "| cell | config | exp/search med / p90 / max | top-10 % share | windows per search 1 / 2 / 3+ | full-grid fallbacks |"
    )
    print("|---|---|---|---|---|---|")
    for bench in BENCHES:
        for config in CONFIGS:
            run = latest(root / "profile", bench, config)
            if run is None:
                continue
            recs = search_records(run)
            ok = [r for r in recs if r["ok"] == 1 and r["ignore_dyn"] == 0 and r["simple"] == 0]
            ex = sorted((r["expanded"] for r in ok), reverse=True)
            top = sum(ex[: max(1, len(ex) // 10)]) / sum(ex) if ex and sum(ex) else float("nan")
            win = Counter(min(3, int(r["windows"])) for r in recs)
            fg = sum(1 for r in recs if r["fullgrid"] == 1)
            print(
                f"| {bench} | {config} | {fmt(pct(ex, 0.5), 0)} / {fmt(pct(ex, 0.9), 0)} / {fmt(max(ex) if ex else None, 0)} | {top:.2f} | {win[1]} / {win[2]} / {win[3]} | {fg} |"
            )
    print()


def section_attempts(root: Path) -> None:
    print("## 3. Attempt structure\n")
    print(
        "Source: `native_negotiated_search` trace lines (one per plain search attempt; kind = plain, plain_retry, "
        "post_ripup_plain / _guided / _crossing_free), `native_repair_probe`, `native_repair_local_ripup`, "
        "`native_repair_global_ripup` lines, and the `A* timing breakdown` of `--debug-timing`. "
        "`first plain ok` = nets whose very first attempt routed; `simple` = attempts with expanded=0 (simple-route path, no kernel).\n"
    )
    print(
        "| cell | config | nets | first plain ok | of which simple | plain_retry (ok) | probes | local rip-ups | global rip-ups | post-rip-up attempts (ok) | normal_route calls ok/fail | rounds |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for bench in BENCHES:
        for config in CONFIGS:
            run = latest(root / "profile", bench, config)
            if run is None:
                continue
            text = (run / "run.log").read_text(errors="replace")
            att = [
                kv_line(line) for line in text.splitlines() if "native_negotiated_search" in line
            ]
            first_seen: dict[str, dict] = {}
            for d in att:
                first_seen.setdefault(d["net"], d)
            nets = len(first_seen)
            first_ok = sum(
                1 for d in first_seen.values() if d["kind"] == "plain" and d["outcome"] == "routed"
            )
            first_simple = sum(
                1
                for d in first_seen.values()
                if d["kind"] == "plain" and d["outcome"] == "routed" and d["expanded"] == "0"
            )
            retry = [d for d in att if d["kind"] == "plain_retry"]
            retry_ok = sum(1 for d in retry if d["outcome"] == "routed")
            post = [d for d in att if d["kind"].startswith("post_ripup")]
            post_ok = sum(1 for d in post if d["outcome"] == "routed")
            probes = len(re.findall(r"native_repair_probe net=", text))
            local = len(re.findall(r"native_repair_local_ripup", text))
            glob = len(re.findall(r"native_repair_global_ripup", text))
            m = re.search(r"normal_route: calls=(\d+), ok=(\d+), fail=(\d+)", text)
            nr = f"{m.group(1)} {m.group(2)}/{m.group(3)}" if m else "-"
            mr = re.findall(r"native_negotiated_done .*?rounds=(\d+)", text)
            rounds = mr[-1] if mr else "-"
            print(
                f"| {bench} | {config} | {nets} | {first_ok} | {first_simple} | {len(retry)} ({retry_ok}) | {probes} | {local} | {glob} | {len(post)} ({post_ok}) | {nr} | {rounds} |"
            )
    print()


def read_metrics(run: Path) -> dict:
    out = {}
    mj = run / "metrics.json"
    if mj.exists():
        out.update(json.loads(mj.read_text()))
    txt = (run / "run.txt").read_text() if (run / "run.txt").exists() else ""
    m = re.search(r"rc=(\d+)", txt)
    out["rc"] = int(m.group(1)) if m else None
    return out


def section_order(root: Path, sources: dict, gate_root: Path | None) -> None:
    print("## 4. Net-order sensitivity\n")
    print(
        "Source: `--net-order random` (uniform permutation of the batch) and `--net-order topological-random` "
        "(depth layers kept, shuffled inside), seeds 1-5 (`PHOTONIC_ROUTER_EXP_NET_ORDER_SEED`); reference = the paper's archived "
        "values (`docs/date2027_table_sources.json`), gate = the default order on this machine (phase 0). "
        "`t` = routing_loop_s. `fail` = rc != 0 or verifier errors or missing routes.\n"
    )
    for policy in ("random", "topological-random"):
        print(f"### {policy}\n")
        print(
            "| cell | config | ref X / L um / t s | gate X / L / t | seed X values | seed L um min / mean / max | L spread vs ref | seed t s min / mean / max | t max/min | fails |"
        )
        print("|---|---|---|---|---|---|---|---|---|---|")
        for bench in BENCHES:
            paper_bench = bench.replace("_flat", "")
            cell = sources.get(paper_bench, {})
            for config in CONFIGS:
                ref = cell.get(PAPER_KEY[config]) or {}
                rows = []
                for seed in range(1, 6):
                    run = latest(root / "order" / policy / f"seed{seed}", bench, config)
                    if run is not None:
                        rows.append((seed, read_metrics(run)))
                if not rows:
                    continue
                gate = (
                    read_metrics(latest(gate_root, bench, config))
                    if gate_root and latest(gate_root, bench, config)
                    else {}
                )

                def ok(m):
                    return (
                        m.get("rc") == 0
                        and int(m.get("photonic_error_count", -1) or 0) == 0
                        and (m.get("routed_record_count") == m.get("expected_route_count"))
                    )

                good = [m for _, m in rows if ok(m)]
                fails = len(rows) - len(good)
                xs = [str(m.get("crossing_count", "?")) for _, m in rows]
                ls = [float(m["gds_length_um"]) for m in good if m.get("gds_length_um") is not None]
                ts = [
                    float(m["routing_loop_s"]) for m in good if m.get("routing_loop_s") is not None
                ]
                refl = ref.get("gds_length_um")
                spread = (
                    f"{(min(ls) / refl - 1) * 100:+.1f} % .. {(max(ls) / refl - 1) * 100:+.1f} %"
                    if ls and refl
                    else "-"
                )
                print(
                    f"| {bench} | {config} | {ref.get('crossing_count', '-')} / {fmt(refl, 1)} / {fmt(ref.get('routing_loop_s'), 2)} | "
                    f"{gate.get('crossing_count', '-')} / {fmt(gate.get('gds_length_um'), 1)} / {fmt(gate.get('routing_loop_s'), 2)} | "
                    f"{' '.join(xs)} | {fmt(min(ls) if ls else None, 1)} / {fmt(statistics.mean(ls) if ls else None, 1)} / {fmt(max(ls) if ls else None, 1)} | {spread} | "
                    f"{fmt(min(ts) if ts else None, 2)} / {fmt(statistics.mean(ts) if ts else None, 2)} / {fmt(max(ts) if ts else None, 2)} | "
                    f"{fmt(max(ts) / min(ts) if ts and min(ts) > 0 else None, 2)} | {fails} |"
                )
        print()


def main() -> int:
    root = Path(sys.argv[1])
    sources = json.loads(Path(sys.argv[2]).read_text())["sources"]
    gate_root = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    section_profile(root)
    section_heuristic(root)
    section_attempts(root)
    section_order(root, sources, gate_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
