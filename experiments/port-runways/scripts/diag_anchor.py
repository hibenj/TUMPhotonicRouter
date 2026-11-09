"""Inspect the runway anchors of one net before dispatch: which foreign
static cells cover the anchor cells / the cells ahead of the anchors."""

from __future__ import annotations

import os
import runpy
import sys

sys.path.insert(0, os.getcwd())
bench, net_name, *flags = sys.argv[1:]
import photonic_router.routing.dispatch as dispatch

orig = dispatch.dispatch_native_routing


class Stop(Exception):
    pass


def patched(settings, state, jobs):
    job = next(
        j
        for j in jobs
        if getattr(j, "net_name", None) == net_name or str(getattr(j, "net_id", "")) == net_name
    )
    specs = (f"{job.inst1},{job.port1}", f"{job.inst2},{job.port2}")
    print("job", job.net_name if hasattr(job, "net_name") else job.net_id, specs)
    by_spec = state.fanout_stub_static_cells_by_spec
    static = getattr(state, "static_blocked_cells_before_port_reservations", set())
    for spec in specs:
        a = state.fanout_anchor_by_port_spec.get(spec)
        print("--", spec, "runway" if spec in state.port_runway_anchor_specs else "fanout/none")
        if a is None:
            print("   no anchor")
            continue
        print(f"   anchor=({a.state_x},{a.state_y}) angle={a.physical_angle} center={a.center_um}")
        print(f"   stub_cells={sorted(a.stub_center_cells)}")
        print(f"   centerline={[tuple(round(c, 3) for c in p) for p in a.stub_centerline_um]}")
        own = by_spec.get(spec, set())
        foreign = {}
        for other, cells in by_spec.items():
            if other == spec:
                continue
            hit = cells & (set(a.stub_center_cells) | {(a.state_x, a.state_y)})
            if hit:
                foreign[other] = sorted(hit)
        print(f"   foreign stub static cells on my stub/anchor: {foreign}")
        print(f"   anchor in static_blocked: {(a.state_x, a.state_y) in static}")
        # cells ahead of the anchor along heading
        import math

        ang = math.radians(a.physical_angle * 45 if a.physical_angle < 8 else a.physical_angle)
        dx, dy = round(math.cos(ang)), round(math.sin(ang))
        ahead = [(a.state_x + k * dx, a.state_y + k * dy) for k in range(1, 6)]
        print(
            f"   ahead cells={ahead} static={[c in static for c in ahead]} foreign={[c in (state.fanout_stub_static_cells - own) for c in ahead]}"
        )
    # neighbours: other specs of the same instance
    for spec in specs:
        inst = spec.split(",")[0]
        for other, a in sorted(state.fanout_anchor_by_port_spec.items()):
            if other.startswith(inst + ",") and other != spec:
                print(
                    f"   sibling {other}: anchor=({a.state_x},{a.state_y}) stub={sorted(a.stub_center_cells)} inflated_n={len(by_spec.get(other, ()))}"
                )
    raise Stop()


dispatch.dispatch_native_routing = patched
sys.argv = ["routing_flow.py", bench, *flags]
try:
    runpy.run_path("routing_flow.py", run_name="__main__")
except (SystemExit, Stop):
    pass
except Exception:
    import traceback

    traceback.print_exc()
