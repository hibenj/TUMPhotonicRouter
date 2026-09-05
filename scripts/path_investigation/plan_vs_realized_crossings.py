"""Topology crossing plan vs the realized crossings of the last run of a benchmark
(reads build/verification/<bench>_crossing_verification.json). Usage:
.venv/bin/python scripts/path_investigation/plan_vs_realized_crossings.py benes_32x32
(contribution 1 design measurement, 2026-09-04)."""

import json, sys
from collections import Counter

sys.path.insert(0, ".")
from routing_flow import load_benchmark
from benchmark_metadata import load_benchmark_metadata
from photonic_router.topology_analysis import analyze_schematic_topology
from photonic_router.crossing_plan import build_crossing_plan

b = sys.argv[1]
sch = load_benchmark(b)
md = load_benchmark_metadata(b, schematic=sch)
topo = analyze_schematic_topology(
    sch,
    node_depths=md.get("node_depths") or None,
    node_ranks=md.get("node_ranks") or None,
    edge_ranks=md.get("edge_ranks") or None,
)
plan = build_crossing_plan(topo)
plan_pairs = Counter(frozenset((e.edge_a.net_name, e.edge_b.net_name)) for e in plan.events)
d = json.load(open(f"build/verification/{b}_crossing_verification.json"))
real_pairs = Counter(frozenset((c["net_name_a"], c["net_name_b"])) for c in d["crossings"])
hit = sum(min(n, real_pairs[p]) for p, n in plan_pairs.items())
print(
    f"{b}: plan events={sum(plan_pairs.values())} stages={len(plan.stages)} | realized={sum(real_pairs.values())} | realized pairs also in plan={hit} | realized NOT in plan={sum(real_pairs.values()) - hit} | plan pairs never realized={sum(plan_pairs.values()) - hit}"
)
miss = [tuple(sorted(p)) for p in real_pairs if p not in plan_pairs][:8]
print("  unplanned examples:", miss)
