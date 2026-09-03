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

    def collinear_runs(poly: db.Polygon):
        """Merge consecutive collinear edges (route polygons are finely sampled:
        a straight diagonal is many 11 um edges) into (x0, y0, x1, y1) runs."""
        pts = [(pt.x * dbu, pt.y * dbu) for pt in poly.each_point_hull()]
        if len(pts) < 2:
            return
        n = len(pts)
        # find a corner to start from so a run is not split at index 0
        def direction(i):
            x0, y0 = pts[i]; x1, y1 = pts[(i + 1) % n]
            dx, dy = x1 - x0, y1 - y0
            l = (dx * dx + dy * dy) ** 0.5
            return (round(dx / l, 3), round(dy / l, 3)) if l > 1e-9 else None
        start = 0
        for i in range(n):
            if direction(i) != direction((i - 1) % n):
                start = i
                break
        run_start = pts[start]; run_dir = direction(start)
        for k in range(1, n + 1):
            i = (start + k) % n
            d = direction(i)
            if d != run_dir:
                x0, y0 = run_start; x1, y1 = pts[i]
                yield x0, y0, x1, y1
                run_start = pts[i]; run_dir = d

    for li in ly.layer_indexes():
        if ly.get_info(li).to_s() != a.layer:
            continue
        for sh in top.begin_shapes_rec(li):
            s = sh.shape()
            if not (s.is_path() or s.is_polygon() or s.is_box()):
                continue
            poly = s.polygon.transformed(sh.trans())
            for x0, y0, x1, y1 in collinear_runs(poly):
                dx, dy = x1 - x0, y1 - y0
                if a.dir == "v" and abs(dx) < 1e-6 and abs(dy) >= a.min_len:
                    pos, lo, hi = x0, min(y0, y1), max(y0, y1)
                elif a.dir == "h" and abs(dy) < 1e-6 and abs(dx) >= a.min_len:
                    pos, lo, hi = y0, min(x0, x1), max(x0, x1)
                elif a.dir == "d" and abs(abs(dx) - abs(dy)) < 0.05 and abs(dx) >= a.min_len / 1.4142:
                    if a.x and not (a.x[0] <= min(x0, x1) and max(x0, x1) <= a.x[1]):
                        continue
                    if a.y and not (a.y[0] <= min(y0, y1) and max(y0, y1) <= a.y[1]):
                        continue
                    key = (round(min(x0, x1), 1), round(min(y0, y1), 1), round(max(x0, x1), 1))
                    runs[key] = max(runs.get(key, 0.0), (dx * dx + dy * dy) ** 0.5)
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
                key = (round(pos, 2), round(lo), round(hi))
                runs[key] = max(runs.get(key, 0.0), hi - lo)

    if a.dir == "d":
        # merge the two edges of one waveguide (offset ~0.35 um along both axes)
        items = sorted(runs.items())
        shown: list[tuple[float, float, float, float]] = []
        for (x0, y0, x1), length in items:
            if shown and abs(shown[-1][0] - x0) <= 0.6 and abs(shown[-1][1] - y0) <= 0.6 and abs(shown[-1][3] - length) <= 1.0:
                continue
            shown.append((x0, y0, x1, length))
            print(f"diag from x={x0:.1f} y={y0:.1f}  dx={x1 - x0:.0f} um  length={length:.0f} um")
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
