"""Milestone 1 feasibility simulation: distributed crossing structure on shortest-path lanes.

Per crossing stage: every lane is an octile-shortest path between its natural
entry row and exit row ([h] diag [h] diag [h] for |dy| <= band, diag-vertical-diag
otherwise). A per-stage solver picks the free parameters so that exactly the
planned pairs cross, at 90 degrees, with legal spacing. Exact segment geometry,
no shapely needed (all segments are axis-aligned or 45 degrees).
"""
from __future__ import annotations
import importlib, math, sys, time, json
import gdsfactory as gf
from benchmark_metadata import load_benchmark_metadata
from translation import preplaced_crossing_grids as pcg
from translation.layout_from_schematic import layout_from_schematic

PORT_STRAIGHT = 6.0    # straight at each port before the first bend (um)
LANE_CLEAR = 5.0       # min centerline distance between lanes away from their crossing
X_SPACING = 10.0       # min distance between crossing centers (8 um footprint + gap)
BEND_TO_X = 8.0        # min distance bend vertex <-> crossing center on the same lane
X_EXCLUDE = 7.0        # around a crossing the two crossing lanes may come closer
ENDPOINT_MARGIN = 10.0 # min distance crossing center <-> lane endpoint (port straight + footprint half)
STEP = 2.0             # candidate grid (one routing cell)
FAN_LANE_SPACING = 22.0  # dense source fan-out lane spacing (11 cells, static stubs)
FAN_LENGTH = 24.0      # x consumed by the two-bend stubs before the lane starts
DENSE_MIN_PORTS = 3
NODE_BUDGET = 120000
EPS = 1e-6

def seg_dir(s):
    (x0,y0),(x1,y1) = s; dx,dy = x1-x0, y1-y0; n = math.hypot(dx,dy)
    return (dx/n, dy/n) if n > EPS else (0.0,0.0)

def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def proper_intersection(s, t):
    """Point where segments s,t cross properly; 'overlap' if collinear overlap; None otherwise."""
    p,q = s; r,w = t
    d1 = cross(r,w,p); d2 = cross(r,w,q); d3 = cross(p,q,r); d4 = cross(p,q,w)
    if abs(d1) < EPS and abs(d2) < EPS:  # collinear
        # overlap test on projection
        ax = [p[0],q[0]]; bx = [r[0],w[0]]; ay=[p[1],q[1]]; by=[r[1],w[1]]
        if max(min(ax),min(bx)) < min(max(ax),max(bx)) - EPS or max(min(ay),min(by)) < min(max(ay),max(by)) - EPS:
            return 'overlap'
        return None
    if ((d1 > EPS and d2 < -EPS) or (d1 < -EPS and d2 > EPS)) and ((d3 > EPS and d4 < -EPS) or (d3 < -EPS and d4 > EPS)):
        # intersection point
        x1,y1 = p; x2,y2 = q; x3,y3 = r; x4,y4 = w
        den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        t_ = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
        return (x1 + t_*(x2-x1), y1 + t_*(y2-y1))
    return None

def point_seg_dist(pt, s):
    (x0,y0),(x1,y1) = s; px,py = pt; dx,dy = x1-x0, y1-y0; L2 = dx*dx+dy*dy
    if L2 < EPS: return math.hypot(px-x0,py-y0), (x0,y0)
    t = max(0.0, min(1.0, ((px-x0)*dx + (py-y0)*dy)/L2))
    cx,cy = x0+t*dx, y0+t*dy
    return math.hypot(px-cx,py-cy), (cx,cy)

def seg_dist(s, t):
    best = (math.inf, None)
    for pt in s:
        d,c = point_seg_dist(pt, t)
        if d < best[0]: best = (d, pt)
    for pt in t:
        d,c = point_seg_dist(pt, s)
        if d < best[0]: best = (d, c)
    return best

def perpendicular(s, t):
    a = seg_dir(s); b = seg_dir(t); return abs(a[0]*b[0] + a[1]*b[1]) < 1e-6

def segs(poly): return [(poly[i], poly[i+1]) for i in range(len(poly)-1)]

def yrange(poly):
    ys = [p[1] for p in poly]; return min(ys), max(ys)

class Lane:
    def __init__(self, net, x_entry, x_exit, ys, yt, src_inst, tgt_inst):
        self.net=net; self.x_entry=x_entry; self.x_exit=x_exit; self.ys=ys; self.yt=yt
        self.src=src_inst; self.tgt=tgt_inst
        self.D = abs(yt-ys); self.s = 1.0 if yt>ys else -1.0; self.W = x_exit - x_entry
        self.poly=None; self.family=None; self.column=None; self.bends=0
    def candidates(self):
        """Yield (poly, family, column_x, bends) -- every shape is [h0] diag [h_mid|vertical] diag [h1]."""
        D,W,s = self.D,self.W,self.s; xe,xx,ys,yt = self.x_entry,self.x_exit,self.ys,self.yt
        out=[]
        if D < EPS:
            out.append(([(xe,ys),(xx,ys)], 'straight', 0.5*(xe+xx), 0)); return out
        g = 4.0 if W <= 150 else 8.0
        G = sorted({0.0} | {k*g for k in range(int(W//g)+1)})
        def add(pts, fam, col, bends):
            poly=[pts[0]]
            for q in pts[1:]:
                if abs(q[0]-poly[-1][0]) > EPS or abs(q[1]-poly[-1][1]) > EPS: poly.append(q)
            out.append((poly, fam, col, bends))
        if D <= W + EPS:
            slack = W - D
            for h0 in G:
                if h0 > slack + EPS: break
                x1 = xe+h0; x2 = x1+D
                add([(xe,ys),(x1,ys),(x2,yt),(xx,yt)], 'diag', 0.5*(x1+x2), 2)          # single diagonal
                for h1 in G:
                    hm = slack - h0 - h1
                    if hm < g - EPS: continue
                    for frac in (0.5, 0.3, 0.7):
                        d1 = round(frac*D/STEP)*STEP; d2 = D - d1
                        if d1 < STEP or d2 < STEP: continue
                        x1=xe+h0; x2=x1+d1; x3=x2+hm; x4=x3+d2
                        add([(xe,ys),(x1,ys),(x2,ys+s*d1),(x3,ys+s*d1),(x4,yt),(xx,yt)], 'diag-h-diag', 0.5*(x1+x4), 4)
            return out
        for h0 in G:
            for h1 in G:
                rem = W - h0 - h1
                if rem < 8.0 - EPS: break
                for frac in (0.5, 0.0, 1.0, 0.25, 0.75):
                    d1 = round(frac*rem/STEP)*STEP; d2 = rem - d1
                    xc = xe + h0 + d1
                    add([(xe,ys),(xe+h0,ys),(xc,ys+s*d1),(xc,yt-s*d2),(xc+d2,yt),(xx,yt)], 'diag-vert-diag', xc, 4 if (d1>EPS and d2>EPS) else 3)
        return out

def pair_ok(A_poly, B_poly, need, share_src, share_tgt, A_lane, B_lane):
    """Check lane A against placed lane B. Returns (ok, crossing_points)."""
    ya=yrange(A_poly); yb=yrange(B_poly)
    if ya[0] > yb[1] + LANE_CLEAR or yb[0] > ya[1] + LANE_CLEAR:
        return (True if need == 0 else 'far'), []
    SA=segs(A_poly); SB=segs(B_poly); pts=[]
    for sa in SA:
        for sb in SB:
            p = proper_intersection(sa, sb)
            if p == 'overlap': return 'overlap', []
            if p is not None:
                if not perpendicular(sa, sb): return 'oblique', []
                pts.append(p)
    if len(pts) != need: return f'count{len(pts)}vs{need}', []
    for sa in SA:
        for sb in SB:
            d, c = seg_dist(sa, sb)
            if d < LANE_CLEAR - EPS:
                if any(math.hypot(c[0]-q[0], c[1]-q[1]) <= X_EXCLUDE for q in pts): continue
                # sibling exemption inside the port straights
                if share_src and c[0] <= max(A_lane.x_entry, B_lane.x_entry) + PORT_STRAIGHT + STEP: continue
                if share_tgt and c[0] >= min(A_lane.x_exit, B_lane.x_exit) - PORT_STRAIGHT - STEP: continue
                return 'clear', []
    return True, pts

def bends_ok(poly, pts):
    for v in poly[1:-1]:
        for q in pts:
            if math.hypot(v[0]-q[0], v[1]-q[1]) < BEND_TO_X - EPS: return False
    for v in (poly[0], poly[-1]):
        for q in pts:
            if math.hypot(v[0]-q[0], v[1]-q[1]) < ENDPOINT_MARGIN - EPS: return False
    return True

ORDER_VARIANTS = [
    ("widest-first,top-down", lambda l: (l.D > EPS, -l.D, -l.ys)),
    ("widest-first,bottom-up", lambda l: (l.D > EPS, -l.D, l.ys)),
    ("top-down", lambda l: (l.D > EPS, -l.ys)),
    ("bottom-up", lambda l: (l.D > EPS, l.ys)),
    ("narrowest-first", lambda l: (l.D > EPS, l.D, -l.ys)),
]

def solve_stage(lanes, planned):
    total=0
    for name, key in ORDER_VARIANTS:
        for l in lanes: l.poly=None
        ok, order, placed, xpts, nodes = _solve_stage(lanes, planned, key)
        total += nodes
        if ok:
            print(f"   solved with order variant {name}")
            return ok, order, placed, xpts, total
    return ok, order, placed, xpts, total

def _solve_stage(lanes, planned, order_key):
    """DFS with conflict-directed backjumping over the lanes."""
    order = sorted(lanes, key=order_key)  # stationary lanes first (pure constraints)
    placed=[]; xpts=[]  # (point, idx_a, idx_b)
    nodes=[0]; deepest=[0, {}]
    def try_place(i):
        """Returns None on success, else the set of placed indices the failure depends on."""
        if i == len(order): return None
        L = order[i]
        cands = L.candidates()
        used_cols = [p.column for p in placed]
        cx = 0.5*(L.x_entry+L.x_exit)
        src_sib = any(P.src == L.src for P in lanes if P is not L)
        tgt_sib = any(P.tgt == L.tgt for P in lanes if P is not L)
        def pref(c):
            poly,fam,col,b = c
            h0 = poly[1][0]-poly[0][0] if abs(poly[1][1]-poly[0][1]) < EPS and len(poly) > 2 else 0.0
            h1 = poly[-1][0]-poly[-2][0] if abs(poly[-1][1]-poly[-2][1]) < EPS and len(poly) > 2 else 0.0
            sib = (h0 if src_sib else 0.0) + (h1 if tgt_sib else 0.0)
            spread = min([abs(col-u) for u in used_cols], default=1e9)
            extra = round(poly_len(poly) - octile_min(L), 1)
            return (b, extra, sib, -min(spread, 60.0), abs(col-cx))
        cands.sort(key=pref)
        if i > deepest[0]: deepest[0]=i; deepest[1]={}
        reasons = deepest[1] if i == deepest[0] else {}
        conflicts=set()
        for poly,fam,col,b in cands:
            nodes[0]+=1
            if nodes[0] > NODE_BUDGET: return set(range(i))
            ok=True; mypts=[]
            for j,P in enumerate(placed):
                need = 1 if frozenset((L.net,P.net)) in planned else 0
                good, pts = pair_ok(poly, P.poly, need, L.src==P.src, L.tgt==P.tgt, L, P)
                if good is not True:
                    key=f"{good}:{P.net}"; reasons[key]=reasons.get(key,0)+1; ok=False; conflicts.add(j); break
                if pts:
                    if not bends_ok(P.poly, pts):
                        key=f"bendB:{P.net}"; reasons[key]=reasons.get(key,0)+1; ok=False; conflicts.add(j); break
                    mypts += [(q, j) for q in pts]
            if not ok: continue
            if not bends_ok(poly, [q for q,_ in mypts]):
                reasons['bendA']=reasons.get('bendA',0)+1; conflicts.update(j for _,j in mypts); continue
            bad=False
            for a in range(len(mypts)):
                for bb in range(a+1,len(mypts)):
                    if math.hypot(mypts[a][0][0]-mypts[bb][0][0], mypts[a][0][1]-mypts[bb][0][1]) < X_SPACING-EPS:
                        bad=True; conflicts.add(mypts[a][1]); conflicts.add(mypts[bb][1])
                for q,ia,ib in xpts:
                    if math.hypot(mypts[a][0][0]-q[0], mypts[a][0][1]-q[1]) < X_SPACING-EPS:
                        bad=True; conflicts.update((ia,ib,mypts[a][1]))
            if bad:
                reasons['xspacing']=reasons.get('xspacing',0)+1; continue
            L.poly=poly; L.family=fam; L.column=col; L.bends=b
            placed.append(L); xpts.extend((q, j, i) for q,j in mypts)
            deeper = try_place(i+1)
            if deeper is None: return None
            placed.pop(); del xpts[len(xpts)-len(mypts):]
            if i not in deeper:
                return deeper          # backjump: this lane is not involved
            conflicts |= (deeper - {i})
            if nodes[0] > NODE_BUDGET: return set(range(i))
        return conflicts
    res = try_place(0)
    ok = res is None
    if not ok:
        top = sorted(deepest[1].items(), key=lambda kv:-kv[1])[:6]
        print(f"   deepest lane index {deepest[0]} ({order[deepest[0]].net}, D={order[deepest[0]].D:.1f} W={order[deepest[0]].W:.1f}); top reasons: {top}")
    return ok, order, placed, [q for q,_,_ in xpts], nodes[0]

def stage_lanes(name, s, lay, ports, bbox, st):
    lanes=[]
    # dense source groups: > DENSE_MIN_PORTS ports of one instance in this stage -> fan-out rows
    by_src={}
    for e in st.initial_edge_order: by_src.setdefault(e.source.instance, []).append(e)
    fan_rows={}; fan_x={}
    for inst, edges in by_src.items():
        if len(edges) < DENSE_MIN_PORTS: continue
        es = sorted(edges, key=lambda e: ports[(e.source.instance,e.source.port)][1])
        n=len(es); lower=es[:n//2]; upper=es[n//2:]
        y_low = ports[(lower[-1].source.instance, lower[-1].source.port)][1]
        y_up = ports[(upper[0].source.instance, upper[0].source.port)][1]
        for rank,e in enumerate(reversed(lower)):
            fan_rows[e.net_name] = y_low - rank*FAN_LANE_SPACING - 0.5*FAN_LANE_SPACING
            fan_x[e.net_name] = FAN_LENGTH + rank*STEP
        for rank,e in enumerate(upper):
            fan_rows[e.net_name] = y_up + rank*FAN_LANE_SPACING + 0.5*FAN_LANE_SPACING
            fan_x[e.net_name] = FAN_LENGTH + rank*STEP
    for e in st.initial_edge_order:
        sx,sy = ports[(e.source.instance,e.source.port)]; tx,ty = ports[(e.target.instance,e.target.port)]
        if e.net_name in fan_rows:
            ys = fan_rows[e.net_name]; x_entry = sx + fan_x[e.net_name]
        else:
            ys = sy; x_entry = sx + PORT_STRAIGHT
        lanes.append(Lane(e.net_name, x_entry, tx - PORT_STRAIGHT, ys, ty, e.source.instance, e.target.instance))
    return lanes

def octile_min(l):
    return l.W + l.D - min(l.D, l.W)*(2-math.sqrt(2))

def poly_len(poly):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in segs(poly))

def svg_stage(path, lanes, xpts, planned):
    xs=[p[0] for l in lanes for p in l.poly]; ys=[p[1] for l in lanes for p in l.poly]
    x0,x1,y0,y1 = min(xs)-20, max(xs)+20, min(ys)-20, max(ys)+20
    sc = 2.0
    W=(x1-x0)*sc; H=(y1-y0)*sc
    def X(x): return (x-x0)*sc
    def Y(y): return (y1-y)*sc
    out=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" viewBox="0 0 {W:.0f} {H:.0f}"><rect width="100%" height="100%" fill="white"/>']
    colors=["#1f77b4","#d62728","#2ca02c","#9467bd","#ff7f0e","#8c564b","#e377c2","#17becf","#bcbd22","#7f7f7f"]
    for i,l in enumerate(lanes):
        pts=" ".join(f"{X(p[0]):.1f},{Y(p[1]):.1f}" for p in l.poly)
        out.append(f'<polyline points="{pts}" fill="none" stroke="{colors[i%len(colors)]}" stroke-width="1.5"/>')
        out.append(f'<text x="{X(l.poly[0][0])-4:.1f}" y="{Y(l.poly[0][1])+3:.1f}" font-size="7" text-anchor="end" fill="{colors[i%len(colors)]}">{l.net}</text>')
    for q in xpts:
        out.append(f'<rect x="{X(q[0])-4*sc:.1f}" y="{Y(q[1])-4*sc:.1f}" width="{8*sc:.1f}" height="{8*sc:.1f}" fill="none" stroke="black" stroke-width="1" transform="rotate(45 {X(q[0]):.1f} {Y(q[1]):.1f})"/>')
    out.append('</svg>'); open(path,"w").write("\n".join(out))

def run(name, want_svg=(), outdir="."):
    bm=importlib.import_module(f"benchmarks.{name}"); s=bm.build_schematic(); lay=layout_from_schematic(s)
    ports={}; bbox={}
    for ref in lay.insts:
        b=ref.dbbox(); bbox[ref.name]=(b.left,b.right)
        for p in ref.ports: ports[(ref.name,p.name)]=(float(p.dcenter[0]),float(p.dcenter[1]))
    plan=pcg.build_crossing_plan_for_benchmark(s, load_benchmark_metadata(name, schematic=s))
    rows=[]
    for key, st in sorted(plan.stages.items()):
        if not st.events: continue
        lanes = stage_lanes(name, s, lay, ports, bbox, st)
        planned={frozenset((ev.edge_a.net_name, ev.edge_b.net_name)) for ev in st.events}
        t0=time.time(); ok, order, placed, xpts, nodes = solve_stage(lanes, planned); dt=time.time()-t0
        # verification of the whole placement (independent re-check)
        realized=0; bad=0
        if ok:
            for i in range(len(placed)):
                for j in range(i+1,len(placed)):
                    A,B=placed[i],placed[j]; need=1 if frozenset((A.net,B.net)) in planned else 0
                    good,pts = pair_ok(A.poly,B.poly,need,A.src==B.src,A.tgt==B.tgt,A,B)
                    if not good: bad+=1
                    realized+=len(pts)
        extra = sum(poly_len(l.poly)-octile_min(l) for l in placed) if ok else float('nan')
        fam = {}
        for l in placed: fam[l.family]=fam.get(l.family,0)+1
        band = min(l.x_exit for l in lanes) - max(l.x_entry for l in lanes)
        cols = sorted(l.column for l in placed if l.family=='diag-vert-diag')
        mincol = min([b-a for a,b in zip(cols,cols[1:])], default=float('inf'))
        failed = None if ok else order[len(placed)].net
        row=dict(bench=name, stage=str(key), lanes=len(lanes), planned=len(planned), ok=ok, realized=realized, bad_pairs=bad,
                 nodes=nodes, time_s=round(dt,1), extra_len_um=round(extra,1) if ok else None, families=fam, band=round(band,1),
                 min_column_gap=round(mincol,1) if ok else None, failed_lane=failed, placed=len(placed))
        rows.append(row)
        print(json.dumps(row), flush=True)
        if ok and str(key) in want_svg:
            svg_stage(f"{outdir}/sps_{name}_{key[0]}_{key[1]}.svg", placed, xpts, planned)
    return rows

if __name__ == "__main__":
    gf.gpdk.PDK.activate()
    from benchmarks.benes import register_benes_cells; register_benes_cells()
    names = sys.argv[1].split(",") if len(sys.argv)>1 else ["benes_8x8"]
    svg = set(x.strip() for x in sys.argv[2].split(";")) if len(sys.argv)>2 else set()
    outdir = sys.argv[3] if len(sys.argv)>3 else "."
    for n in names: run(n, svg, outdir)
