"""Column grid (axis-aligned '+' crossings only) budget per multiportmmi stage.

Lane = entry horizontal (row = static fan-out anchor row, approx.) -> vertical at
its column -> target horizontal. Column order from the pairwise crossing
requirements (flipped pairs cross once either way, non-flipped interleaved
pairs force an order). Lanes may share a column when their vertical spans are
disjoint and no order constraint ties them. Reports columns, width, band.
"""
import importlib, math, sys, itertools
import gdsfactory as gf
from benchmark_metadata import load_benchmark_metadata
from translation import preplaced_crossing_grids as pcg
from translation.layout_from_schematic import layout_from_schematic
gf.gpdk.PDK.activate()
EPS=1e-6; FAN_SPACING=22.0; FAN_X=24.0; DENSE=3
def between(v,a,b): lo,hi=min(a,b),max(a,b); return lo+EPS<v<hi-EPS
def crossings_if_left(A,B):
    n=0
    if between(B[0],A[0],A[1]): n+=1
    if between(A[1],B[0],B[1]): n+=1
    return n
def run(name, pitch):
    bm=importlib.import_module(f"benchmarks.{name}"); s=bm.build_schematic(); lay=layout_from_schematic(s)
    ports={(r.name,p.name):(float(p.dcenter[0]),float(p.dcenter[1])) for r in lay.insts for p in r.ports}
    bbox={r.name:(r.dbbox().left,r.dbbox().right) for r in lay.insts}
    plan=pcg.build_crossing_plan_for_benchmark(s, load_benchmark_metadata(name, schematic=s))
    for key, st in sorted(plan.stages.items()):
        if not st.events: continue
        lanes=[e.net_name for e in st.initial_edge_order]
        # fan-out rows for dense sources
        by_src={}
        for e in st.initial_edge_order: by_src.setdefault(e.source.instance,[]).append(e)
        row={}; fanx={}
        for inst,es in by_src.items():
            es=sorted(es,key=lambda e:ports[(e.source.instance,e.source.port)][1])
            if len(es)>=DENSE:
                n=len(es); lower=es[:n//2]; upper=es[n//2:]
                ylo=ports[(lower[-1].source.instance,lower[-1].source.port)][1]; yup=ports[(upper[0].source.instance,upper[0].source.port)][1]
                for r,e in enumerate(reversed(lower)): row[e.net_name]=ylo-(r+0.5)*FAN_SPACING; fanx[e.net_name]=FAN_X+2*r
                for r,e in enumerate(upper): row[e.net_name]=yup+(r+0.5)*FAN_SPACING; fanx[e.net_name]=FAN_X+2*r
            else:
                for e in es: row[e.net_name]=ports[(e.source.instance,e.source.port)][1]; fanx[e.net_name]=0.0
        geo={e.net_name:(row[e.net_name], ports[(e.target.instance,e.target.port)][1]) for e in st.initial_edge_order}
        x0=max(bbox[e.source.instance][1] for e in st.initial_edge_order); x1=min(bbox[e.target.instance][0] for e in st.initial_edge_order)
        planned={frozenset((ev.edge_a.net_name,ev.edge_b.net_name)) for ev in st.events}
        moving=[n for n in lanes if abs(geo[n][0]-geo[n][1])>EPS]
        before={n:set() for n in moving}; tied=set(); bad=0
        for a,b in itertools.combinations(moving,2):
            need=1 if frozenset((a,b)) in planned else 0
            la=crossings_if_left(geo[a],geo[b]); lb=crossings_if_left(geo[b],geo[a])
            if la==need and lb==need: continue
            if la==need: before[b].add(a); tied.add(frozenset((a,b)))
            elif lb==need: before[a].add(b); tied.add(frozenset((a,b)))
            else: bad+=1
        order=[]; rem=set(moving); cyc=False
        while rem:
            ready=[n for n in rem if before[n]<=set(order)]
            if not ready: cyc=True; break
            ready.sort(key=lambda n:-abs(geo[n][0]-geo[n][1])); order.append(ready[0]); rem.remove(ready[0])
        # greedy column sharing in order
        columns=[]  # list of lists of lanes
        for n in order:
            span=(min(geo[n]),max(geo[n]))
            placed=False
            for col in columns:
                if all((max(geo[m])<span[0]-8 or min(geo[m])>span[1]+8) and frozenset((n,m)) not in tied for m in col):
                    # sharing also must not reorder: n must be after all lanes in earlier columns that must be before it -> ok since we scan in order
                    col.append(n); placed=True; break
            if not placed: columns.append([n])
        fan=max(fanx.values()); band=x1-x0
        width=len(columns)*pitch+fan+10  # +10: target run-in
        print(f"  {name} stage {key}: lanes={len(lanes)} moving={len(moving)} crossings={len(planned)} order_ok={not cyc and bad==0} columns={len(columns)} (no sharing {len(moving)}) fan={fan:.0f} width@{pitch:.0f}um={width:.0f} band={band:.0f} {'FITS' if width<=band else 'TOO WIDE'}")
for n in ("multiportmmi_8x8","multiportmmi_16x16","multiportmmi_32x32"):
    run(n, float(sys.argv[1]) if len(sys.argv)>1 else 12.0)
