"""Slice 5-0, third measurement: run routing_flow.py in-process and read the
realized centerlines of every net: per port the lateral residue to the cell
centre axis, the geometry of today's correction S (forward extent, lateral
shift), the arc length until the route leaves the port's lane (first real
turn), and per net the length the correction added (total - base)."""

from __future__ import annotations

import json
import math
import runpy
import statistics as st
import sys
from pathlib import Path

bench, out_path, *flags = sys.argv[1:]
import photonic_router.routing.session as sess

captured = {}
orig_run = sess._RouteNetsRustSession.run


def patched_run(self):
    layout, dbg = orig_run(self)
    captured["spec"] = dbg.realization_grid_spec
    recs = []
    for r in dbg.routed_net_records:
        recs.append(
            dict(
                net=r.net_name,
                src=r.source_port_center_um,
                tgt=r.target_port_center_um,
                src_deg=r.source_port_orientation_deg,
                tgt_deg=r.target_port_orientation_deg,
                total=r.total_length_um,
                base=r.base_total_length_um,
                err=r.endpoint_correction_error,
                cl=[list(p) for p in r.corrected_centerline_um],
            )
        )
    captured["records"] = recs
    return layout, dbg


sess._RouteNetsRustSession.run = patched_run
sys.argv = ["routing_flow.py", bench, *flags]
try:
    runpy.run_path("routing_flow.py", run_name="__main__")
except SystemExit:
    pass
Path(out_path).write_text(json.dumps(captured))

# ---- analysis
spec = captured["spec"]
recs = captured["records"]
print(f"grid_spec={spec}")
# spec guess: (width, height, grid_size_um, origin_x, origin_y)
g = float(spec[2])
ox = float(spec[3])
oy = float(spec[4])


def unit(deg):
    a = math.radians(deg)
    return (round(math.cos(a), 9), round(math.sin(a), 9))


def analyze_end(port, deg, cl):
    if port is None or deg is None or len(cl) < 2:
        return None
    h = unit(deg)
    l = (-h[1], h[0])
    if math.dist(cl[0], port) > math.dist(cl[-1], port):
        cl = cl[::-1]
    # residue of the lateral coordinate against the cell-centre axes
    if abs(h[0]) > 0.5:
        c, o = port[1], oy
    else:
        c, o = port[0], ox
    r = ((c - o) / g - 0.5) % 1.0
    d = min(r, 1 - r) * g
    s = [0.0]
    lat = []
    fwd = []
    for i, p in enumerate(cl):
        v = (p[0] - port[0], p[1] - port[1])
        lat.append(v[0] * l[0] + v[1] * l[1])
        fwd.append(v[0] * h[0] + v[1] * h[1])
        if i:
            s.append(s[-1] + math.dist(cl[i - 1], p))
    lane_exit = next((s[i] for i in range(len(cl)) if abs(lat[i]) > 1.5), None)
    # correction S: deviation starts, then heading parallel to h again before lane exit
    s_fwd = s_lat = None
    i0 = next((i for i in range(len(cl)) if abs(lat[i]) > 0.02), None)
    if i0 is not None and (lane_exit is None or s[i0] < lane_exit):
        for i in range(i0, len(cl) - 1):
            seg = (cl[i + 1][0] - cl[i][0], cl[i + 1][1] - cl[i][1])
            n = math.hypot(*seg)
            if n < 1e-9:
                continue
            cross = (seg[0] * h[1] - seg[1] * h[0]) / n
            if abs(cross) < 1e-6 and fwd[i] > 0:
                s_fwd, s_lat = fwd[i], lat[i]
                break
    return dict(
        d=d, lane_exit=lane_exit, s_fwd=s_fwd, s_lat=s_lat, dist_port_to_cl=math.dist(cl[0], port)
    )


ends = []
added = []
total = 0.0
nerr = 0
nocl = 0
for r in recs:
    total += r["total"] or 0
    if r["err"]:
        nerr += 1
    if r["base"] is not None and r["total"] is not None:
        added.append(r["total"] - r["base"])
    if not r["cl"]:
        nocl += 1
        continue
    for port, deg in ((r["src"], r["src_deg"]), (r["tgt"], r["tgt_deg"])):
        e = analyze_end(port, deg, r["cl"])
        if e:
            ends.append(e)
n = len(ends)
straight_axis = sum(1 for e in ends if e["d"] < 1e-6)
with_s = [e for e in ends if e["s_fwd"] is not None]
le = [e["lane_exit"] for e in ends if e["lane_exit"] is not None]


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None


print(
    f"{bench} nets={len(recs)} no_centerline={nocl} corr_errors={nerr} ports={n} "
    f"on_axis={straight_axis} ({100 * straight_axis / max(n, 1):.0f} %) "
    f"residue_classes={sorted(set(round(e['d'], 3) for e in ends))}"
)
print(
    f"  correction S present on {len(with_s)} ports ({100 * len(with_s) / max(n, 1):.0f} %): "
    f"forward extent median={q([e['s_fwd'] for e in with_s], 0.5)} p90={q([e['s_fwd'] for e in with_s], 0.9)} "
    f"max={max((e['s_fwd'] for e in with_s), default=None)}; |lateral shift| median={q([abs(e['s_lat']) for e in with_s], 0.5)} max={max((abs(e['s_lat']) for e in with_s), default=None)}"
)
print(
    f"  lane exit (first real turn) arc length: ports with exit={len(le)} of {n}; "
    f"<10um={sum(1 for x in le if x < 10)} <16um={sum(1 for x in le if x < 16)} <20um={sum(1 for x in le if x < 20)} <40um={sum(1 for x in le if x < 40)} median={q(le, 0.5):.1f} p10={q(le, 0.1):.1f}"
)
print(
    f"  correction added length: nets_with_base={len(added)} sum={sum(added):.1f} um of total {total:.1f} um "
    f"({100 * sum(added) / max(total, 1e-9):.2f} %), per corrected net mean={st.mean([a for a in added if abs(a) > 1e-6] or [0]):.2f} max={max(added, default=0):.2f} min={min(added, default=0):.2f}, nets_with_added>0={sum(1 for a in added if a > 1e-6)}"
)
