"""List long straight waveguide runs (vertical, horizontal or 45-degree) in a
routed GDS, read from polygon edges with klayout -- the geometry the crossing
rules are judged on. Adjacent parallel runs closer than 5 cells (10 um at
grid 2 um) cannot both be crossed by one straight (reservation windows
+-half_size must stay disjoint).

Usage:
  python scripts/path_investigation/gds_runs.py GDS [--dir v|h|d] [--min-len UM]
                                                [--x XMIN XMAX] [--y YMIN YMAX] [--layer 1/0]
Prints one line per run: direction, position, extent, length; for vertical/horizontal
runs it also flags neighbours closer than --pitch (default 10 um).
"""
from __future__ import annotations

import argparse

import klayout.db as db


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("gds")
    ap.add_argument("--dir", choices=("v", "h", "d"), default="v")
    ap.add_argument("--min-len", type=float, default=60.0)
    ap.add_argument("--x", type=float, nargs=2)
    ap.add_argument("--y", type=float, nargs=2)
    ap.add_argument("--layer", default="1/0")
    ap.add_argument("--pitch", type=float, default=10.0, help="flag parallel neighbours closer than this (um)")
    a = ap.parse_args()

    ly = db.Layout()
    ly.read(a.gds)
    top = ly.top_cell()
    dbu = ly.dbu
    runs: dict[tuple[float, float, float], float] = {}
    for li in ly.layer_indexes():
        if ly.get_info(li).to_s() != a.layer:
            continue
        for sh in top.begin_shapes_rec(li):
            s = sh.shape()
            if not (s.is_path() or s.is_polygon() or s.is_box()):
                continue
            poly = s.polygon.transformed(sh.trans())
            for e in poly.each_edge():
                dx, dy = e.dx() * dbu, e.dy() * dbu
                if a.dir == "v" and dx == 0 and abs(dy) >= a.min_len:
                    pos, lo, hi = e.x1 * dbu, min(e.y1, e.y2) * dbu, max(e.y1, e.y2) * dbu
                elif a.dir == "h" and dy == 0 and abs(dx) >= a.min_len:
                    pos, lo, hi = e.y1 * dbu, min(e.x1, e.x2) * dbu, max(e.x1, e.x2) * dbu
                elif a.dir == "d" and abs(abs(dx) - abs(dy)) < 1e-6 and abs(dx) >= a.min_len / 1.4142:
                    # diagonal: report start/end instead of a position
                    x0, y0, x1, y1 = e.x1 * dbu, e.y1 * dbu, e.x2 * dbu, e.y2 * dbu
                    if a.x and not (a.x[0] <= min(x0, x1) and max(x0, x1) <= a.x[1]):
                        continue
                    if a.y and not (a.y[0] <= min(y0, y1) and max(y0, y1) <= a.y[1]):
                        continue
                    key = (round(min(x0, x1), 1), round(min(y0, y1), 1), round(max(x0, x1), 1))
                    runs[key] = max(runs.get(key, 0.0), (abs(dx) ** 2 + abs(dy) ** 2) ** 0.5)
                    continue
                else:
                    continue
                if a.dir == "v":
                    if a.x and not (a.x[0] <= pos <= a.x[1]):
                        continue
                    if a.y and not (a.y[0] <= lo and hi <= a.y[1]):
                        continue
                else:
                    if a.y and not (a.y[0] <= pos <= a.y[1]):
                        continue
                    if a.x and not (a.x[0] <= lo and hi <= a.x[1]):
                        continue
                # waveguide edges come in pairs (width ~0.5 um): merge to one run per 1 um bucket
                key = (round(pos, 2), round(lo), round(hi))
                runs[key] = max(runs.get(key, 0.0), hi - lo)

    if a.dir == "d":
        for (x0, y0, x1), length in sorted(runs.items()):
            print(f"diag from x={x0} y={y0} to x={x1}  length={length:.1f} um")
        return 0

    # Waveguide edges come in pairs (one per side of the ~0.5 um core): merge
    # runs whose position differs by <= 0.6 um and whose extent matches.
    merged: list[list[float]] = []  # [pos, lo, hi]
    for (pos, lo, hi), _length in sorted(runs.items()):
        if merged and abs(merged[-1][0] - pos) <= 0.6 and merged[-1][1] == lo and merged[-1][2] == hi:
            merged[-1][0] = (merged[-1][0] + pos) / 2.0
        else:
            merged.append([pos, lo, hi])
    axis = "x" if a.dir == "v" else "y"
    span = "y" if a.dir == "v" else "x"
    for i, (pos, lo, hi) in enumerate(merged):
        flags = []
        # parallel neighbours = runs at another position whose extent overlaps this one
        for j, (pos2, lo2, hi2) in enumerate(merged):
            if j == i or abs(pos2 - pos) >= a.pitch or abs(pos2 - pos) < 0.7:
                continue
            if min(hi, hi2) - max(lo, lo2) > 0:
                flags.append(f"{abs(pos2 - pos):.0f} um from {axis}={pos2:.1f}")
        flag = f"  <-- parallel within {a.pitch:.0f} um (not both crossable by one straight): " + ", ".join(flags) if flags else ""
        print(f"{axis}={pos:.1f} um  {span}={lo:.0f}..{hi:.0f} ({hi - lo:.0f} um){flag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
