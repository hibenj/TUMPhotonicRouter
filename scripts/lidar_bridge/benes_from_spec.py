"""Build a LiDAR benchmark YAML from a Benes spec written by export_benes_spec.py.

Run inside LiDAR's environment from its benchmarks directory, because it uses
LiDAR's own schematic writer (macro library, placement lists, layout.yml):

    cd ~/Documents/Repositories/working/LiDAR/src/picroute/benchmarks
    ../../../.venv/bin/python <this file> spec.json

The YAML lands in ``<benchmarks>/<design>/<design>.yml`` like LiDAR's own
generated benchmarks.
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.getcwd())
import gdsfactory as gf
from schematic import CustomSchematic

COMPONENTS = {
    "grating_coupler_te": gf.components.grating_coupler_te,
    "mmi2x2": gf.components.mmi2x2,
    "straight_heater_metal": gf.components.straight_heater_metal,
}


def build(spec_path: str) -> str:
    with open(spec_path) as handle:
        spec = json.load(handle)
    name = spec["design"]
    path = os.path.join(os.getcwd(), name, f"{name}.yml")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    se = CustomSchematic(path)
    se.update_settings(design=name, die_area=[[0, 0], spec["die"]], wg_radius=5)
    for inst in spec["instances"]:
        se.add_instance(inst["name"], COMPONENTS[inst["component"]](), iloss=inst["iloss"])
        if inst["orient"] == "FN":
            se.schematic.placements[inst["name"]].mirror = True
        se.update_placement(
            inst["name"], ["FIXED", [inst["x"], inst["y"]], inst["orient"], spec["halo"]]
        )
    for (i1, p1), (i2, p2) in spec["nets"]:
        se.add_net(i1, p1, i2, p2)
    se.commit()
    return path


if __name__ == "__main__":
    print("wrote", build(sys.argv[1]))
