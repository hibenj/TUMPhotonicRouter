"""Template for A/B tables from routing_flow logs + crossing JSONs (expects
<prefix>_<bench>_<mode>.log / .crossing.json next to a <prefix>.summary as
written by the scratchpad runner scripts; adapt S and the file names)."""

import json, re, sys, os

S = os.path.dirname(os.path.abspath(__file__))
rows = []
for line in open(f"{S}/c1ab.summary"):
    b, m, rc, wall = line.split()
    log = open(f"{S}/c1ab_{b}_{m}.log").read()
    rs = re.search(
        r"route search: astar_loop=([\d.]+)s, attempts=(\d+), failures=(\d+), simple=\S+, repairs=(\d+)",
        log,
    )
    ex = re.search(r"expanded=(\d+)", log)
    ap = re.search(r"accepted=(\d+), accepted_planned=(\d+)", log)
    c = json.load(open(f"{S}/c1ab_{b}_{m}.crossing.json"))
    mt = c["metrics"]
    length = sum(r["length_um"] for r in c["route_costs"])
    rows.append(
        (
            b,
            m,
            rc,
            float(wall[5:-1]),
            rs.group(1),
            f"{rs.group(2)}/{rs.group(3)}/{rs.group(4)}",
            int(ex.group(1)),
            mt["crossing_count"],
            mt.get("realized_planned_crossing_count", "-"),
            mt.get("realized_unplanned_crossing_count", "-"),
            mt.get("plan_unrealized_pair_count", "-"),
            round(length, 1),
            mt["error_count"],
            ap.group(2) if ap else "-",
        )
    )
print(
    "| benchmark | mode | rc | wall s | astar s | att/fail/rep | expanded | crossings | planned | unplanned | unrealized | length um | errors | accepted_planned |"
)
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for r in rows:
    print("| " + " | ".join(str(x) for x in r) + " |")
