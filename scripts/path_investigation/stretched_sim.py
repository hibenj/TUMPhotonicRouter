"""Stretched crossing array: slot rows = natural lane rows (siblings spread +-SPREAD),
levels = columns spread over the band, each crossing at (level x, midpoint of the two
swapped slot rows); lane between consecutive crossings = 45-degree arm, vertical, arm."""
import importlib, math, sys
import gdsfactory as gf
from benchmark_metadata import load_benchmark_metadata
from translation import preplaced_crossing_grids as pcg
from translation.layout_from_schematic import layout_from_schematic
gf.gpdk.PDK.activate()
from benchmarks.benes import register_benes_cells; register_benes_cells()
SPREAD=11.0; MARGIN=30.0; BEND_X=3.54; XHALF=4.0; ARM_MIN=4.0; ROW_PITCH_CAP=None

def run(name, svg_stage=None, outdir="."):
    bm=importlib.import_module(f"benchmarks.{name}"); s=bm.build_schematic(); lay=layout_from_schematic(s)
    ports={(r.name,p.name):(float(p.dcenter[0]),float(p.dcenter[1])) for r in lay.insts for p in r.ports}
    bbox={r.name:(r.dbbox().left, r.dbbox().right) for r in lay.insts}
    plan=pcg.build_crossing_plan_for_benchmark(s, load_benchmark_metadata(name, schematic=s))
    for key, st in sorted(plan.stages.items()):
        if not st.events: continue
        movement, crossings = pcg._lane_movement(st)
        levels = max(l for l,*_ in crossings)+1
        lanes=[e.net_name for e in st.initial_edge_order]
        # slot rows: natural source rows, siblings spread
        src_inst={e.net_name:e.source.instance for e in st.initial_edge_order}
        rows={e.net_name: ports[(e.source.instance,e.source.port)][1] for e in st.initial_edge_order}
        by_inst={}
        for n in lanes: by_inst.setdefault(src_inst[n],[]).append(n)
        for inst,ns in by_inst.items():
            if len(ns)==2:
                a,b=sorted(ns,key=lambda n:rows[n]); c=(rows[a]+rows[b])/2; rows[a]=c-SPREAD; rows[b]=c+SPREAD
        slot_row=[rows[n] for n in lanes]  # slot i = initial order i (top to bottom => descending y)
        x0=max(bbox[e.source.instance][1] for e in st.initial_edge_order); x1=min(bbox[e.target.instance][0] for e in st.initial_edge_order)
        band=x1-x0; pitch=(band-2*MARGIN)/levels
        # crossing positions
        xs=[x0+MARGIN+(k+0.5)*pitch for k in range(levels)]
        xpos={}
        for (level, upper_slot, upper, lower) in crossings:
            y=(slot_row[upper_slot]+slot_row[upper_slot+1])/2
            xpos[(level,upper_slot)]=(xs[level],y)
        gaps=sorted({abs(slot_row[i]-slot_row[i+1]) for i in range(len(slot_row)-1)})
        arm=min((pitch/2-BEND_X-1.0)*math.sqrt(2), min(gaps)/2*math.sqrt(2))  # diagonal arm length; x/y extent = arm/sqrt2
        min_arm_ok = arm >= XHALF+ARM_MIN
        # per-lane length vs octile optimum
        extra=0.0; n=0
        for lane in lanes:
            if lane not in movement: continue
            first,last,d = movement[lane]
            r0 = slot_row[lanes.index(lane)]; slot = lanes.index(lane)
            # lane path: horizontal to first arm, then per level: arm, X, arm, vertical to next X ...
            length = band  # horizontals + x-extents approx: count exact below
            ys=r0; xcur=x0; L=0.0
            for lvl in range(first,last+1):
                # crossing this lane takes at this level
                key2=[k for k in xpos if k[0]==lvl and k[1] in (slot, slot-1)]
                # find the crossing involving this lane
                for (level,upper_slot,upper,lower) in crossings:
                    if level==lvl and lane in (upper.net_name, lower.net_name): cx,cy=xpos[(level,upper_slot)]; break
                ax=arm/math.sqrt(2)
                # from (xcur,ys) horizontal to cx-ax-... then arm to crossing, arm out
                L += max(0.0, (cx-ax) - xcur) if lvl==first else 0.0
                if lvl>first:
                    # vertical from previous exit arm end to this entry arm start
                    L += abs((cy - (ax if d>0 else -ax)*1) - ys_exit) - 0  # rough: vertical run
                L += 2*arm  # two arms (in and out) through the crossing (crossing pass length inside arm accounting)
                ys_exit = cy - d*ax  # rough exit y after arm
                xcur = cx+ax
                slot += d
            L += (x1 - xcur)
            D = abs(slot_row[lanes.index(lane)] - slot_row[slot]) if False else abs(r0 - slot_row[slot])
            opt = band + D - min(D,band)*(2-math.sqrt(2))
            extra += L-opt; n+=1
        print(f"  {name} stage {key}: lanes={len(lanes)} crossings={len(crossings)} levels={levels} band={band:.0f} pitch={pitch:.0f} arm={arm:.1f} ({'ok' if min_arm_ok else 'TOO SHORT'}) row gaps={[round(g) for g in gaps]} mean extra length/lane={extra/max(n,1):.0f} um")
        if svg_stage == str(key):
            draw(f"{outdir}/stretched_{name}_{key[0]}_{key[1]}.svg", lanes, slot_row, movement, crossings, xpos, x0, x1, arm)

def draw(path, lanes, slot_row, movement, crossings, xpos, x0, x1, arm):
    ax=arm/math.sqrt(2)
    sc=1.0; ys=slot_row; y0=min(ys)-40; y1=max(ys)+40
    W=(x1-x0+80)*sc; H=(y1-y0)*sc
    X=lambda x:(x-x0+40)*sc; Y=lambda y:(y1-y)*sc
    out=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}"><rect width="100%" height="100%" fill="white"/>']
    cols=["#1f77b4","#d62728","#2ca02c","#9467bd","#ff7f0e","#8c564b","#e377c2","#17becf","#bcbd22","#7f7f7f","#aec7e8","#ffbb78","#98df8a","#ff9896","#c5b0d5","#c49c94"]
    for i,lane in enumerate(lanes):
        pts=[(x0, slot_row[i])]; slot=i
        if lane in movement:
            first,last,d=movement[lane]
            for lvl in range(first,last+1):
                for (level,upper_slot,upper,lower) in crossings:
                    if level==lvl and lane in (upper.net_name, lower.net_name): cx,cy=xpos[(level,upper_slot)]; break
                # entry arm start
                pts.append((cx-ax, cy+d*ax)); pts.append((cx+ax, cy-d*ax))  # through the crossing at 45 deg
                slot+=d
            pts.append((x1, slot_row[slot]))
        else:
            pts.append((x1, slot_row[i]))
        # insert vertical/horizontal connectors: consecutive points differing in both x and y beyond 45 deg get an elbow
        poly=[pts[0]]
        for idx,p in enumerate(pts[1:]):
            q=poly[-1]; dx=p[0]-q[0]; dy=p[1]-q[1]
            if abs(abs(dx)-abs(dy))<1e-6 or abs(dy)<1e-6 or abs(dx)<1e-6: poly.append(p)
            elif idx==0:   # leaving the source port: horizontal first, vertical just before the first arm
                poly.append((p[0], q[1])); poly.append(p)
            else:          # after a crossing exit: vertical first, then horizontal
                poly.append((q[0], p[1])); poly.append(p)
        s=" ".join(f"{X(x):.1f},{Y(y):.1f}" for x,y in poly)
        out.append(f'<polyline points="{s}" fill="none" stroke="{cols[i%len(cols)]}" stroke-width="1.5"/>')
        out.append(f'<text x="{X(x0)-3:.0f}" y="{Y(slot_row[i])+3:.0f}" font-size="8" text-anchor="end">{lane}</text>')
    for (cx,cy) in xpos.values():
        out.append(f'<rect x="{X(cx)-4*sc:.1f}" y="{Y(cy)-4*sc:.1f}" width="{8*sc}" height="{8*sc}" fill="none" stroke="black" transform="rotate(45 {X(cx):.1f} {Y(cy):.1f})"/>')
    out.append('</svg>'); open(path,"w").write("\n".join(out))

if __name__=="__main__":
    names=sys.argv[1].split(","); stage=sys.argv[2] if len(sys.argv)>2 else None; outdir=sys.argv[3] if len(sys.argv)>3 else "."
    for n in names: run(n, stage, outdir)
