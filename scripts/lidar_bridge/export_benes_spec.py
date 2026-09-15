"""Export a Benes benchmark as a placement/netlist spec for LiDAR.

LiDAR (the reference router, ~/Documents/Repositories/working/LiDAR) has no
Benes benchmark. This script writes a JSON description of one of ours -- every
switch expanded into its primitives (two 2x2 MMIs, two heater arms), every
instance with the lower-left corner of its bounding box, and every net
including the switch-internal ones -- which
``scripts/lidar_bridge/benes_from_spec.py`` turns into LiDAR's YAML inside
LiDAR's own environment. Usage (repository root, our venv):

    PYTHONPATH=. .venv/bin/python scripts/lidar_bridge/export_benes_spec.py benes_8x8 out.json
"""

from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path

import gdsfactory as gf

from translation.layout_from_schematic import layout_from_schematic

MARGIN_UM = 100.0  # die margin around the layout bounding box
PLACEMENT_HALO = [10, 10, 10, 10]  # LiDAR's per-instance keep-out, as in its own benchmarks
ILOSS_DB = {"grating_coupler_te": 2.0, "mmi2x2": 0.3, "straight_heater_metal": 0.1}
# switch-internal connections of benchmarks/benes.py::benes_mmi_heater_switch
SWITCH_INTERNAL_NETS = (
    ("mmi_in", "o3", "heater_top", "o1"),
    ("heater_top", "o2", "mmi_out", "o2"),
    ("mmi_in", "o4", "heater_bottom", "o1"),
    ("heater_bottom", "o2", "mmi_out", "o1"),
)
SWITCH_PORT_MAP = {
    "o1": ("mmi_in", "o1"),
    "o2": ("mmi_in", "o2"),
    "o3": ("mmi_out", "o3"),
    "o4": ("mmi_out", "o4"),
}


def _orient(trans: gf.kdb.DCplxTrans) -> str:
    """LEF/DEF orientation of a klayout transform (LiDAR's placement vocabulary).

    A gdsfactory ``Placement(mirror=True)`` (a left-right flip, our input
    couplers) is the klayout transform "rotate 180 and reflect", i.e.
    ``angle == 180`` with the mirror flag: LEF ``FN``."""
    angle = round(trans.angle) % 360
    key = (angle, bool(trans.is_mirror()))
    table = {(0, False): "N", (180, False): "S", (180, True): "FN", (0, True): "FS"}
    if key not in table:
        raise ValueError(f"unsupported transform angle={angle} mirror={key[1]}")
    return table[key]


def export(benchmark: str, out: Path) -> None:
    module = importlib.import_module(f"benchmarks.{benchmark}")
    schematic = module.build_schematic()
    layout = layout_from_schematic(schematic)
    instances: list[dict] = []
    for ref in layout.insts:
        name = str(ref.name)
        cell_name = ref.cell.name
        if cell_name.startswith("benes_mmi_heater_switch"):
            for sub in ref.cell.insts:
                sub_cell = sub.cell.name
                if sub_cell.startswith("mmi2x2"):
                    comp = "mmi2x2"
                elif sub_cell.startswith("straight_heater_metal"):
                    comp = "straight_heater_metal"
                else:
                    continue  # the switch's internal waveguides: LiDAR routes them itself
                trans = ref.dcplx_trans * sub.dcplx_trans
                box = sub.cell.dbbox().transformed(trans)
                instances.append(
                    {
                        "name": f"{name}__{sub.name}",
                        "component": comp,
                        "x": box.left,
                        "y": box.bottom,
                        "orient": _orient(trans),
                        "iloss": ILOSS_DB[comp],
                    }
                )
        else:
            box = ref.dbbox()
            # our IO_COMPONENT; gdsfactory 9 names the cell after the underlying pcell
            comp = "grating_coupler_te"
            if "grating_coupler" not in cell_name:
                raise ValueError(f"unexpected top-level cell {cell_name} for {name}")
            instances.append(
                {
                    "name": name,
                    "component": comp,
                    "x": box.left,
                    "y": box.bottom,
                    "orient": _orient(ref.dcplx_trans),
                    "iloss": ILOSS_DB[comp],
                }
            )
    # shift so that every corner is inside a die starting at (0, 0)
    xmin = min(i["x"] for i in instances) - MARGIN_UM
    ymin = min(i["y"] for i in instances) - MARGIN_UM
    for i in instances:
        i["x"] = round(i["x"] - xmin, 3)
        i["y"] = round(i["y"] - ymin, 3)
    bbox = layout.dbbox()
    die = [round(bbox.right - xmin + MARGIN_UM, 1), round(bbox.top - ymin + MARGIN_UM, 1)]

    nets: list[list[list[str]]] = []
    switch_names = {
        str(r.name) for r in layout.insts if r.cell.name.startswith("benes_mmi_heater_switch")
    }
    for sw in sorted(switch_names):
        for a, pa, b, pb in SWITCH_INTERNAL_NETS:
            nets.append([[f"{sw}__{a}", pa], [f"{sw}__{b}", pb]])

    def endpoint(spec: str) -> list[str]:
        inst, port = spec.split(",")
        if inst in switch_names:
            sub, sub_port = SWITCH_PORT_MAP[port]
            return [f"{inst}__{sub}", sub_port]
        return [inst, port]

    for net in schematic.nets:
        nets.append([endpoint(net.p1), endpoint(net.p2)])
    out.write_text(
        json.dumps(
            {
                "design": benchmark,
                "die": die,
                "halo": PLACEMENT_HALO,
                "instances": instances,
                "nets": nets,
            },
            indent=1,
        )
    )
    print(
        f"{benchmark}: {len(instances)} instances, {len(nets)} nets "
        f"({len(schematic.nets)} routed in our flow + {4 * len(switch_names)} "
        f"switch-internal), die {die}"
    )


if __name__ == "__main__":
    export(sys.argv[1], Path(sys.argv[2]))
