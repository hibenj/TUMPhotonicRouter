"""Route shapes outside the chip: top-level layer-1/0 shapes of a routed GDS vs
the union bbox of all non-route instances (0.6 um slack for the waveguide
half-width). Usage: .venv/bin/python scripts/path_investigation/gds_outside_chip.py build/routed_<bench>.gds
(chip-boundary keepout, 2026-09-04)."""

import sys, klayout.db as db

path = sys.argv[1]
ly = db.Layout()
ly.read(path)
top = ly.top_cell()
li = ly.layer(1, 0)
comp = db.Box()
for inst in top.each_inst():
    if not inst.cell.name.startswith(("straight", "crossing")):
        comp += inst.bbox()
# also top-level non-route shapes? component polygons are in child cells; routes are top-level shapes
out = []
for s in top.each_shape(li):
    b = s.bbox()
    # allow the waveguide half-width (0.5 um) of slack on each side
    if (
        b.right > comp.right + 600
        or b.left < comp.left - 600
        or b.top > comp.top + 600
        or b.bottom < comp.bottom - 600
    ):
        out.append(b.to_s())
print(path, "component bbox", comp.to_s(), "outside route shapes:", len(out))
for o in out[:10]:
    print("  ", o)
