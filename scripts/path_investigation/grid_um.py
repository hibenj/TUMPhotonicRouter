"""Grid cell <-> um conversion for path investigations.

The origin and grid size MUST come from the flow's own print
(`PHOTONIC_ROUTER_TRACE_GRID=1 python routing_flow.py <bench> --debug-stop-after-route 1`
prints `grid: origin=(ox, oy) um size=s um`), never from a hand-derived bbox.

Usage:
  python scripts/path_investigation/grid_um.py OX OY SIZE cell X Y      # cell centre -> um
  python scripts/path_investigation/grid_um.py OX OY SIZE um X_UM Y_UM  # um -> cell
"""

from __future__ import annotations

import sys


def cell_to_um(ox: float, oy: float, size: float, x: int, y: int) -> tuple[float, float]:
    return ox + (x + 0.5) * size, oy + (y + 0.5) * size


def um_to_cell(ox: float, oy: float, size: float, xu: float, yu: float) -> tuple[int, int]:
    return int((xu - ox) // size), int((yu - oy) // size)


def main(argv: list[str]) -> int:
    if len(argv) != 6:
        print(__doc__)
        return 2
    ox, oy, size = (float(v) for v in argv[:3])
    mode = argv[3]
    if mode == "cell":
        x, y = int(argv[4]), int(argv[5])
        xu, yu = cell_to_um(ox, oy, size, x, y)
        print(f"cell ({x},{y}) -> ({xu:.3f}, {yu:.3f}) um")
    elif mode == "um":
        xu, yu = float(argv[4]), float(argv[5])
        x, y = um_to_cell(ox, oy, size, xu, yu)
        print(f"({xu}, {yu}) um -> cell ({x},{y})")
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
