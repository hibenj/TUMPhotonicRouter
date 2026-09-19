"""Metrics of one archived run (see run_and_archive.sh): route count, total
and mean waveguide length, crossing count, verifier error counts.

Usage: summarize_run.py <archive dir>  -- prints one `metrics ...` line and
writes metrics.json next to the reports. Reads only the two verification
JSONs, so it also works on any build/verification/ pair.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def summarize(archive: Path) -> dict:
    photonic = next(archive.glob("*_photonic_verification.json"), None)
    crossing = next(archive.glob("*_crossing_verification.json"), None)
    out: dict = {}
    if photonic is not None:
        p = json.loads(photonic.read_text())
        out["photonic_error_count"] = p.get("error_count")
        out["routed_record_count"] = p.get("routed_record_count")
        out["expected_route_count"] = p.get("expected_route_count")
        grids = (p.get("metrics") or {}).get("preplaced_crossing_grids") or {}
        if grids:
            # contribution 2: the crossings are the placed components
            out["crossing_count"] = grids.get("crossing_component_count")
            out["preplaced_expected_crossings"] = grids.get("expected_crossing_count")
    if crossing is not None:
        c = json.loads(crossing.read_text())
        m = c.get("metrics", {})
        out["crossing_error_count"] = c.get("error_count")
        out["crossing_count"] = m.get("crossing_count")
        out["illegal_crossing_count"] = m.get("illegal_crossing_count")
        out["total_physical_insertion_loss"] = m.get("total_physical_insertion_loss")
        costs = c.get("route_costs") or []
        lengths = [float(r["length_um"]) for r in costs if r.get("length_um") is not None]
        if lengths:
            out["route_cost_count"] = len(lengths)
            out["total_length_um"] = round(sum(lengths), 3)
            out["mean_length_um"] = round(sum(lengths) / len(lengths), 3)
            out["max_length_um"] = round(max(lengths), 3)
        crossings_per_net: dict[str, int] = {}
        for x in c.get("crossings") or []:
            for key in ("net_name_a", "net_name_b"):
                name = x.get(key)
                if name is not None:
                    crossings_per_net[name] = crossings_per_net.get(name, 0) + 1
        if crossings_per_net:
            out["max_crossings_per_net"] = max(crossings_per_net.values())
            out["nets_with_crossings"] = len(crossings_per_net)
    out.update(log_metrics(archive / "run.log"))
    out.update(gds_lengths(archive))
    (archive / "metrics.json").write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    return out


WAVEGUIDE_LAYER = "1/0"  # the generic PDK's WG layer
WAVEGUIDE_WIDTH_UM = 0.5
CROSSING_CELL_PREFIX = "crossing"  # crossing components (inline or pre-placed tiles, corners, links)


def gds_lengths(archive: Path) -> dict:
    """Waveguide length measured on the routed layout (owner rule 2026-09-19:
    one length measure for every configuration). Routed waveguides are the
    polygons drawn directly into the top cell; crossing elements are the
    `crossing*` instances (the inline crossing components of the lidar modes,
    the tiles, corners and static links of contribution 2). Length = polygon
    area on the WG layer / the 0.5 um strip width, so wider crossing bodies
    count a little long, identically in every configuration. Benchmark
    components and static fan-out stubs are not counted."""
    gds = next(archive.glob("routed_*.gds"), None)
    if gds is None:
        return {}
    try:
        import klayout.db as kdb
    except ImportError:
        return {}
    layout = kdb.Layout()
    layout.read(str(gds))
    dbu = layout.dbu
    top = next(cell for cell in layout.each_cell() if cell.is_top())
    wg = next(
        (index for index in layout.layer_indexes() if str(layout.get_info(index)) == WAVEGUIDE_LAYER),
        None,
    )
    if wg is None:
        return {}
    to_um = lambda area_dbu: area_dbu * dbu * dbu / WAVEGUIDE_WIDTH_UM  # noqa: E731
    routed = to_um(sum(shape.polygon.area() for shape in top.shapes(wg).each()))
    cell_area: dict[int, float] = {}
    crossing = 0.0
    for inst in top.each_inst():
        cell = layout.cell(inst.cell_index)
        if not cell.name.startswith(CROSSING_CELL_PREFIX):
            continue
        if inst.cell_index not in cell_area:
            cell_area[inst.cell_index] = sum(
                it.shape().polygon.area() for it in cell.begin_shapes_rec(wg)
            )
        crossing += to_um(cell_area[inst.cell_index])
    return {
        "gds_routed_length_um": round(routed, 3),
        "gds_crossing_length_um": round(crossing, 3),
        "gds_length_um": round(routed + crossing, 3),
    }


_LOG_PATTERNS = {
    # `route search: astar_loop=2201.9764s, attempts=1077, failures=62, simple=465/895, repairs=26, deferred=0`
    "search_astar_loop_s": r"route search: astar_loop=([0-9.]+)s",
    "search_attempts": r"route search: .*?attempts=(\d+)",
    "search_failures": r"route search: .*?failures=(\d+)",
    "search_repairs": r"route search: .*?repairs=(\d+)",
    "search_deferred": r"route search: .*?deferred=(\d+)",
    "search_simple_routes": r"route search: .*?simple=(\d+)/",
    # `native_negotiated_done t=1136.4 rounds=3 global_ripups=1 local_ripups=17 ...` (PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG)
    "negotiated_rounds": r"native_negotiated_done .*?rounds=(\d+)",
    "negotiated_global_ripups": r"native_negotiated_done .*?global_ripups=(\d+)",
    "negotiated_local_ripups": r"native_negotiated_done .*?local_ripups=(\d+)",
    "negotiated_probe_guided": r"native_negotiated_done .*?probe_guided=(\d+)",
    "negotiated_crossing_free": r"native_negotiated_done .*?crossing_free=(\d+)",
    "negotiated_wall_s": r"native_negotiated_done t=([0-9.]+)",
    # contribution 2: `Grids: 209 placed, 209 crossing component(s) ... (7.63s)`
    "preplaced_grid_count": r"Grids: (\d+) placed",
    "preplaced_crossing_components": r"Grids: \d+ placed, (\d+) crossing component",
    "preplaced_build_s": r"Grids: .*?\(([0-9.]+)s\)",
    # routing time of ours (owner rule 2026-09-19: both engines report the same
    # phase): `Optical routing stage time (net routing + PLM + realization)` =
    # obstacle map + search with repairs + path-length matching + route
    # realization; contribution 2 adds the structure construction
    # (`preplaced_build_s`) in `routing_time_s` below. Excluded on both sides:
    # reading the benchmark, LiDAR's bitmap / our layout translation, the
    # verifier and the GDS export.
    "routing_stage_s": r"Optical routing stage time \(net routing \+ PLM \+ realization\): ([0-9.]+) s",
    # the routing loop alone (obstacle map + search with repairs, no path-length
    # matching / realization): `net routing phase (obstacles + A* + repairs): 21.4549 s`
    "net_routing_phase_s": r"net routing phase \(obstacles \+ A\* \+ repairs\): ([0-9.]+) s",
    # original LiDAR: `detailed routing takes 101.85 seconds` = DrGridRoute.solve()
    # (net order, DRC init, routing loop with rip-ups, post-processing with
    # Euler alignment and crossing components, evaluation)
    "lidar_detailed_routing_s": r"detailed routing takes ([0-9.]+) seconds",
}


def lidar_routing_loop_seconds(text: str) -> float | None:
    """Original LiDAR's routing loop from its log timestamps: `Start Detailed
    Routing` (drmanager.py) to `DrGridRoute succeed` / `DrGridRoute fail`
    (drgridroute.py, after the last rip-up iteration, before post-processing
    and evaluation, which take < 0.5 % of `detailed routing takes`). None for
    runs that never reached the end line (timeout)."""
    import datetime
    import re

    clean = re.sub(r"\x1b\[[0-9;]*m", "", text)
    stamp = r"(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d+)"
    start = re.search(stamp + r" - drmanager\.py\[line:\d+\] - INFO: Start Detailed Routing", clean)
    end = re.search(stamp + r" - drgridroute\.py\[line:\d+\] - (?:INFO|WARNING): DrGridRoute (?:succeed|fail)", clean)
    if start is None or end is None:
        return None
    parse = lambda v: datetime.datetime.strptime(v, "%Y-%m-%d %H:%M:%S,%f")  # noqa: E731
    return round((parse(end.group(1)) - parse(start.group(1))).total_seconds(), 3)


def log_metrics(log_path: Path) -> dict:
    """Search-effort counters from the run log: attempts, failures, repairs,
    A* loop seconds (every run), the negotiated engine's rounds and rip-ups
    (runs with PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG), contribution 2's grid
    counts and structure build time. Counters that occur several times
    (several routing batches) are summed; seconds too."""
    import re

    if not log_path.exists():
        return {}
    text = log_path.read_text(errors="replace")
    out: dict = {}
    for key, pattern in _LOG_PATTERNS.items():
        values = re.findall(pattern, text)
        if not values:
            continue
        if key.endswith("_s"):
            out[key] = round(sum(float(v) for v in values), 3)
        elif key in ("negotiated_rounds", "preplaced_grid_count", "preplaced_crossing_components"):
            out[key] = max(int(v) for v in values)
        else:
            out[key] = sum(int(v) for v in values)
    if "routing_stage_s" in out:
        out["routing_time_s"] = round(out["routing_stage_s"] + out.get("preplaced_build_s", 0.0), 3)
    if "net_routing_phase_s" in out:
        # the paper's routing time (owner 2026-09-19: the actual routing time on
        # both sides): the routing loop, for contribution 2 with the structure
        # construction that replaces part of the search
        out["routing_loop_s"] = round(out["net_routing_phase_s"] + out.get("preplaced_build_s", 0.0), 3)
    loop = lidar_routing_loop_seconds(text)
    if loop is not None:
        out["lidar_routing_loop_s"] = loop
    braids = len(re.findall(r"native_repair_braid_result .*?keep=true", text))
    if braids:
        out["braid_repairs_kept"] = braids
    # --verbose-routes: `ok astar length=2464.565um ...` / `ok simple length=146.000um ...`
    route_lengths = [float(v) for v in re.findall(r"^ok (?:astar|simple) length=([0-9.]+)um", text, re.M)]
    if route_lengths and "total_length_um" not in out:
        out["route_cost_count"] = len(route_lengths)
        out["total_length_um"] = round(sum(route_lengths), 3)
        out["mean_length_um"] = round(sum(route_lengths) / len(route_lengths), 3)
        out["max_length_um"] = round(max(route_lengths), 3)
        out["length_source"] = "verbose_routes"
    # contribution 2: `column-grid: ok, +14968 um, ...` / x-array lines carry the structure's waveguide length
    structure_um = [float(v) for v in re.findall(r"ok, \+([0-9.]+) um", text)]
    if structure_um:
        out["structure_length_um"] = round(sum(structure_um), 3)
        if "total_length_um" in out:
            out["total_length_with_structures_um"] = round(out["total_length_um"] + out["structure_length_um"], 3)
    return out


def main() -> int:
    archive = Path(sys.argv[1])
    out = summarize(archive)
    print("metrics " + " ".join(f"{k}={v}" for k, v in sorted(out.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
