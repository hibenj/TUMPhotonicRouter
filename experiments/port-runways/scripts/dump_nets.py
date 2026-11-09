"""Run routing_flow.py in-process and dump the records of named nets
(route cells, base/corrected centerlines, ports, correction error) before
the verification raise."""

from __future__ import annotations

import json
import os
import runpy
import sys

sys.path.insert(0, os.getcwd())
bench, out_path, names, *flags = sys.argv[1:]
names = set(names.split(","))
import photonic_router.routing.session as sess

orig_run = sess._RouteNetsRustSession.run


def patched_run(self):
    layout, dbg = orig_run(self)
    out = {"spec": dbg.realization_grid_spec, "nets": []}
    for r in dbg.routed_net_records:
        if r.net_name in names:
            ro = r.route_obj
            cells = None
            for attr in ("cells", "path", "states", "centerline_cells"):
                v = getattr(ro, attr, None)
                if v is not None:
                    try:
                        cells = [list(map(int, c[:2])) if not isinstance(c, int) else c for c in v][
                            :4000
                        ]
                        break
                    except Exception:
                        cells = str(v)[:2000]
                        break
            out["nets"].append(
                dict(
                    net=r.net_name,
                    net_id=r.net_id,
                    src=r.source_port_center_um,
                    tgt=r.target_port_center_um,
                    src_deg=r.source_port_orientation_deg,
                    tgt_deg=r.target_port_orientation_deg,
                    total=r.total_length_um,
                    base=r.base_total_length_um,
                    err=r.endpoint_correction_error,
                    cl=[list(p) for p in r.corrected_centerline_um],
                    route_attrs=[a for a in dir(ro) if not a.startswith("_")][:40],
                    cells=cells,
                )
            )
    from pathlib import Path

    Path(out_path).write_text(json.dumps(out))
    print("dumped", len(out["nets"]), "nets to", out_path)
    return layout, dbg


sess._RouteNetsRustSession.run = patched_run
sys.argv = ["routing_flow.py", bench, *flags]
try:
    runpy.run_path("routing_flow.py", run_name="__main__")
except SystemExit:
    pass
