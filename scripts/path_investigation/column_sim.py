"""Feasibility simulation of a distributed column crossing structure.

Every moving lane of a crossing stage: horizontal from its source row to its
column x, vertical to its target row, horizontal to the target. Crossings are
vertical-vs-horizontal (90 deg). The column order must realize exactly the
planned crossing pairs. Reports feasibility, column counts vs band width and
row spacing per stage for the six crossing benchmarks.
"""
import importlib, math, sys, itertools, collections
import gdsfactory as gf
from benchmark_metadata import load_benchmark_metadata
from translation import preplaced_crossing_grids as pcg
from translation.layout_from_schematic import layout_from_schematic
gf.gpdk.PDK.activate()
from benchmarks.benes import register_benes_cells; register_benes_cells()
EPS=1e-6
def between(v, a, b):  # strictly inside the open interval
    lo,hi=min(a,b),max(a,b); return lo+EPS < v < hi-EPS
def crossings_if_left(A,B):
    """Crossings when A's column is LEFT of B's column. A=(ys,yt), B=(ys,yt)."""
    n=0
    if between(B[0], A[0], A[1]): n+=1          # A vertical x B entry horizontal
    if between(A[1], B[0], B[1]): n+=1          # B vertical x A exit horizontal
    return n
def run(name, col_pitch=16.0):
    bm=importlib.import_module(f"benchmarks.{name}"); s=bm.build_schematic(); lay=layout_from_schematic(s)
    ports={}; bbox={}
    for ref in lay.insts:
        b=ref.dbbox(); bbox[ref.name]=(b.left,b.right)
        for p in ref.ports: ports[(ref.name,p.name)]=(float(p.dcenter[0]),float(p.dcenter[1]))
    plan=pcg.build_crossing_plan_for_benchmark(s, load_benchmark_metadata(name, schematic=s))
    for key, st in sorted(plan.stages.items()):
        if not st.events: continue
        lanes=[e.net_name for e in st.initial_edge_order]
        geo={}
        for e in st.initial_edge_order:
            geo[e.net_name]=(ports[(e.source.instance,e.source.port)][1], ports[(e.target.instance,e.target.port)][1])
        x0=max(bbox[e.source.instance][1] for e in st.initial_edge_order); x1=min(bbox[e.target.instance][0] for e in st.initial_edge_order)
        planned={frozenset((ev.edge_a.net_name, ev.edge_b.net_name)) for ev in st.events}
        moving=[n for n in lanes if abs(geo[n][0]-geo[n][1])>EPS]
        before={n:set() for n in moving}  # before[a] = set of b that must be LEFT of a
        infeasible=[]; free=0
        for a,b in itertools.combinations(moving,2):
            need=1 if frozenset((a,b)) in planned else 0
            la=crossings_if_left(geo[a],geo[b]); lb=crossings_if_left(geo[b],geo[a])
            ok_a = la==need; ok_b = lb==need
            if ok_a and ok_b: free+=1
            elif ok_a: before[b].add(a)
            elif ok_b: before[a].add(b)
            else: infeasible.append((a,b,need,la,lb))
        # stationary lanes: pure horizontals; their planned crossings must come from the other lane's vertical
        stat_bad=0
        for n in lanes:
            if n in moving: continue
            for m in moving:
                need=1 if frozenset((n,m)) in planned else 0
                has=1 if between(geo[n][0], geo[m][0], geo[m][1]) else 0
                if has!=need: stat_bad+=1
        # topological order
        order=[]; remaining=set(moving); cyclic=False
        while remaining:
            ready=[n for n in remaining if before[n]<=set(order)]
            if not ready: cyclic=True; break
            ready.sort(key=lambda n: -abs(geo[n][0]-geo[n][1]))  # widest span first among ready (LiDAR-like)
            n=ready[0]; order.append(n); remaining.remove(n)
        # column sharing: greedy interval packing over vertical spans, respecting order constraints only loosely (report both)
        ncol=len(order); width_needed=ncol*col_pitch
        # row spacing: crossings on one column at the rows of the crossed lanes
        min_gap=math.inf
        if not cyclic:
            xs={n:i for i,n in enumerate(order)}
            for a in order:
                rows=[]
                for b in lanes:
                    if b==a: continue
                    if b in xs and xs[b]>xs[a] and between(geo[b][0],geo[a][0],geo[a][1]): rows.append(geo[b][0])
                    if b in xs and xs[b]<xs[a] and between(geo[b][1],geo[a][0],geo[a][1]): rows.append(geo[b][1])
                    if b not in xs and between(geo[b][0],geo[a][0],geo[a][1]): rows.append(geo[b][0])
                rows.sort()
                for r1,r2 in zip(rows,rows[1:]): min_gap=min(min_gap,r2-r1)
        band=x1-x0
        print(f"  {name} stage {key}: lanes={len(lanes)} moving={len(moving)} planned={len(planned)} pair-infeasible={len(infeasible)} stationary-mismatch={stat_bad} cyclic={cyclic} columns={ncol} need={width_needed:.0f} band={band:.0f} {'FITS' if width_needed<=band-2*col_pitch else 'TOO WIDE'} min_row_gap_on_a_column={min_gap:.1f}")
        for a,b,need,la,lb in infeasible[:2]: print(f"      infeasible pair {a} {geo[a]} x {b} {geo[b]} need={need} left={la} right={lb}")
for name in sys.argv[1:] or ("benes_8x8","benes_16x16","benes_32x32","multiportmmi_8x8","multiportmmi_16x16","multiportmmi_32x32"):
    run(name)
