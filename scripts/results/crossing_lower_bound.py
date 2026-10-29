"""Placement-derived lower bound on waveguide crossings (experiment 5e).

Usage:
    crossing_lower_bound.py <benchmark> [<benchmark> ...] [--per-net] [--json PATH]
    crossing_lower_bound.py --all-paper-cells [--json PATH]

No routing runs. The benchmark's placed layout is built the way the flow
does (`layout_from_schematic`); the chip is cut into vertical *bands* whose
walls are the component columns (a column = a maximal set of components
whose x-extents overlap or touch, extended over the full chip height). A
net whose two terminals sit on the two walls of one band and point into it
is *band-confined*. Two band-confined nets of the same band must cross
(an odd number of times) iff their terminals alternate around the band's
boundary, i.e. iff their top-to-bottom order on the left wall differs from
their order on the right wall. The sum over all such pairs is

  * exact for routings that keep every net inside its band, and
  * a lower bound otherwise (a net that leaves its band can only add
    crossings with the nets of the bands it visits).

Nets that do not fit the band model (terminals on the same wall, on
non-adjacent walls, pointing up/down, or into the chip boundary) are
counted and reported; they contribute 0 to the bound.

For comparison the script also builds today's derived crossing plan
(`photonic_router.crossing_plan.build_crossing_plan` on the topology
analysis the flow uses) and reports where its planned pairs differ from the
alternating pairs of the bound.
"""

from __future__ import annotations

import argparse
import importlib
import json
import signal
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

TABLE_SOURCES = REPO_ROOT / "docs" / "date2027_table_sources.json"
PAPER_CONFIGS = ("lidar", "baseline", "contribution1", "contribution2")
ORIENTATION_TOLERANCE_DEG = 1.0
TOUCH_TOLERANCE_UM = 1e-3


# --------------------------------------------------------------------------
# geometry records
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Terminal:
    instance: str
    port: str
    x: float
    y: float
    orientation: float

    @property
    def side(self) -> str:
        """'right' (port points +x), 'left' (points -x) or 'vertical'."""
        angle = self.orientation % 360.0
        if min(angle, 360.0 - angle) <= ORIENTATION_TOLERANCE_DEG:
            return "right"
        if abs(angle - 180.0) <= ORIENTATION_TOLERANCE_DEG:
            return "left"
        return "vertical"


@dataclass(frozen=True)
class NetLink:
    name: str
    a: Terminal
    b: Terminal


@dataclass
class Column:
    index: int
    xmin: float
    xmax: float
    instances: list[str] = field(default_factory=list)


@dataclass
class BandNet:
    net: str
    left_y: float
    right_y: float
    left_recess_um: float
    right_recess_um: float


@dataclass
class Band:
    index: int  # band between column index and index + 1
    nets: list[BandNet] = field(default_factory=list)
    alternating_pairs: list[tuple[str, str]] = field(default_factory=list)
    per_net: Counter[str] = field(default_factory=Counter)
    tie_pairs: int = 0

    @property
    def bound(self) -> int:
        return len(self.alternating_pairs)


@dataclass
class BoundResult:
    benchmark: str
    instance_count: int
    net_count: int
    columns: list[Column]
    bands: dict[int, Band]
    outside: dict[str, list[str]]  # reason -> net names
    outside_detail: dict[str, str]  # net -> detail
    recessed_terminals: int
    max_recess_um: float
    worst_recess: list[tuple[str, str, float]]  # (net, instance.port, recess um)
    layout_s: float

    @property
    def bound(self) -> int:
        return sum(band.bound for band in self.bands.values())

    @property
    def outside_count(self) -> int:
        return sum(len(names) for names in self.outside.values())

    @property
    def per_net(self) -> Counter[str]:
        total: Counter[str] = Counter()
        for band in self.bands.values():
            total.update(band.per_net)
        return total

    @property
    def alternating_pairs(self) -> set[frozenset[str]]:
        return {frozenset(pair) for band in self.bands.values() for pair in band.alternating_pairs}


# --------------------------------------------------------------------------
# layout extraction (the flow's path: build_schematic -> layout_from_schematic)
# --------------------------------------------------------------------------


class LayoutTimeout(Exception):
    pass


def _alarm(_signum: int, _frame: Any) -> None:
    raise LayoutTimeout


def build_placed_layout(benchmark: str, *, time_limit_s: float) -> tuple[Any, Any, float]:
    """Return (schematic, unrouted layout, seconds) exactly as the flow builds them."""
    from gdsfactory.gpdk import get_generic_pdk

    from translation.layout_from_schematic import layout_from_schematic

    get_generic_pdk().activate()
    module = importlib.import_module(f"benchmarks.{benchmark}")
    start = time.perf_counter()
    previous = signal.signal(signal.SIGALRM, _alarm)
    signal.alarm(int(max(1, time_limit_s)))
    try:
        schematic = module.build_schematic()
        layout = layout_from_schematic(schematic)
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous)
    return schematic, layout, time.perf_counter() - start


def extract_geometry(
    schematic: Any, layout: Any
) -> tuple[dict[str, tuple[float, float, float, float]], list[NetLink]]:
    boxes: dict[str, tuple[float, float, float, float]] = {}
    ports: dict[tuple[str, str], Terminal] = {}
    for inst in layout.insts:
        name = str(inst.name)
        box = inst.dbbox()
        boxes[name] = (float(box.left), float(box.bottom), float(box.right), float(box.top))
        for port in inst.ports:
            center = port.center
            ports[(name, str(port.name))] = Terminal(
                instance=name,
                port=str(port.name),
                x=float(center[0]),
                y=float(center[1]),
                orientation=float(port.orientation),
            )

    links: list[NetLink] = []
    for net_name, bundle in schematic.netlist.routes.items():
        for p1_spec, p2_spec in bundle.links.items():
            inst1, port1 = str(p1_spec).split(",", 1)
            inst2, port2 = str(p2_spec).split(",", 1)
            links.append(NetLink(str(net_name), ports[(inst1, port1)], ports[(inst2, port2)]))
    return boxes, links


# --------------------------------------------------------------------------
# bands and the bound
# --------------------------------------------------------------------------


def cluster_columns(boxes: dict[str, tuple[float, float, float, float]]) -> list[Column]:
    ordered = sorted(boxes.items(), key=lambda item: (item[1][0], item[1][2], item[0]))
    columns: list[Column] = []
    for name, (xmin, _ymin, xmax, _ymax) in ordered:
        if columns and xmin <= columns[-1].xmax + TOUCH_TOLERANCE_UM:
            current = columns[-1]
            current.xmax = max(current.xmax, xmax)
            current.instances.append(name)
            continue
        columns.append(Column(index=len(columns), xmin=xmin, xmax=xmax, instances=[name]))
    return columns


def _band_of(terminal: Terminal, column_index: int, column_count: int) -> int | None:
    """Band index the terminal points into, or None for the chip boundary."""
    side = terminal.side
    if side == "right":
        band = column_index
    elif side == "left":
        band = column_index - 1
    else:
        return None
    if band < 0 or band >= column_count - 1:
        return None
    return band


def compute_bound(
    benchmark: str,
    boxes: dict[str, tuple[float, float, float, float]],
    links: list[NetLink],
    *,
    layout_s: float,
) -> BoundResult:
    columns = cluster_columns(boxes)
    column_of = {name: column.index for column in columns for name in column.instances}
    bands: dict[int, Band] = {}
    outside: dict[str, list[str]] = {}
    outside_detail: dict[str, str] = {}
    recessed = 0
    max_recess = 0.0
    recess_records: list[tuple[str, str, float]] = []

    def reject(net: str, reason: str, detail: str) -> None:
        outside.setdefault(reason, []).append(net)
        outside_detail[net] = detail

    for link in links:
        a, b = link.a, link.b
        col_a, col_b = column_of[a.instance], column_of[b.instance]
        if a.side == "vertical" or b.side == "vertical":
            reject(link.name, "vertical_port", f"orientations {a.orientation}/{b.orientation}")
            continue
        band_a = _band_of(a, col_a, len(columns))
        band_b = _band_of(b, col_b, len(columns))
        if band_a is None or band_b is None:
            reject(link.name, "chip_boundary", f"columns {col_a}/{col_b}, a port points off-chip")
            continue
        if band_a != band_b:
            skipped = abs(col_a - col_b) - 1
            reject(
                link.name,
                "non_adjacent_walls",
                f"columns {col_a}->{col_b} (skips {skipped} column(s))",
            )
            continue
        if a.side == b.side:
            reject(link.name, "same_wall", f"both terminals point {a.side} in band {band_a}")
            continue
        left, right = (a, b) if a.side == "right" else (b, a)
        band = bands.setdefault(band_a, Band(index=band_a))
        left_recess = columns[band_a].xmax - left.x
        right_recess = right.x - columns[band_a + 1].xmin
        for terminal, recess in ((left, left_recess), (right, right_recess)):
            if recess > TOUCH_TOLERANCE_UM:
                recessed += 1
                max_recess = max(max_recess, recess)
                recess_records.append((link.name, f"{terminal.instance}.{terminal.port}", recess))
        band.nets.append(BandNet(link.name, left.y, right.y, left_recess, right_recess))

    for band in bands.values():
        nets = band.nets
        for i in range(len(nets)):
            for j in range(i + 1, len(nets)):
                d_left = nets[i].left_y - nets[j].left_y
                d_right = nets[i].right_y - nets[j].right_y
                if d_left == 0.0 or d_right == 0.0:
                    band.tie_pairs += 1
                    continue
                if (d_left > 0) != (d_right > 0):
                    band.alternating_pairs.append((nets[i].net, nets[j].net))
                    band.per_net[nets[i].net] += 1
                    band.per_net[nets[j].net] += 1

    return BoundResult(
        benchmark=benchmark,
        instance_count=len(boxes),
        net_count=len(links),
        columns=columns,
        bands=bands,
        outside=outside,
        outside_detail=outside_detail,
        recessed_terminals=recessed,
        max_recess_um=max_recess,
        worst_recess=sorted(recess_records, key=lambda r: -r[2]),
        layout_s=layout_s,
    )


# --------------------------------------------------------------------------
# today's derived plan, for comparison
# --------------------------------------------------------------------------


@dataclass
class PlanComparison:
    event_count: int
    stage_count: int
    common: int
    plan_only: list[tuple[str, str, str]]  # (net a, net b, reason)
    bound_only: list[tuple[str, str, str]]
    error: str | None = None


def derived_plan_pairs(
    benchmark: str, schematic: Any
) -> tuple[set[frozenset[str]], int, int, dict[str, tuple[int, int]]]:
    """Planned pairs, event count, stage count and (source, target) depth per net."""
    from photonic_router.crossing_plan import build_crossing_plan
    from photonic_router.topology_analysis import analyze_schematic_topology

    module = importlib.import_module(f"benchmarks.{benchmark}")
    topology = analyze_schematic_topology(
        schematic,
        node_depths=getattr(module, "NODE_DEPTHS", None) or None,
        node_ranks=getattr(module, "NODE_RANKS", None) or None,
        edge_ranks=getattr(module, "EDGE_RANKS", None) or None,
    )
    plan = build_crossing_plan(topology)
    pairs = {frozenset((e.edge_a.net_name, e.edge_b.net_name)) for e in plan.events}
    stage_of = {
        key.net_name: (rank.source_depth, rank.target_depth)
        for key, rank in topology.edge_ranks.items()
    }
    return pairs, len(plan.events), len(plan.stages), stage_of


def compare_with_plan(benchmark: str, schematic: Any, result: BoundResult) -> PlanComparison:
    try:
        plan_pairs, event_count, stage_count, stage_of = derived_plan_pairs(benchmark, schematic)
    except Exception as exc:  # the plan is a comparison aid, never a blocker
        return PlanComparison(0, 0, 0, [], [], error=f"{type(exc).__name__}: {exc}")
    bound_pairs = result.alternating_pairs
    band_of_net = {net.net: band.index for band in result.bands.values() for net in band.nets}

    def describe(pair: frozenset[str], *, in_plan: bool) -> tuple[str, str, str]:
        a, b = sorted(pair)
        reasons = []
        for net in (a, b):
            if net in result.outside_detail:
                reasons.append(f"{net}: outside band model ({result.outside_detail[net]})")
        if not reasons and in_plan:
            if band_of_net.get(a) != band_of_net.get(b):
                reasons.append(
                    f"different bands {band_of_net.get(a)}/{band_of_net.get(b)} but same plan "
                    f"stage {stage_of.get(a)}"
                )
            else:
                reasons.append("same band, terminals do not alternate (plan ranks differ)")
        if not reasons and not in_plan:
            if stage_of.get(a) != stage_of.get(b):
                reasons.append(f"plan stages differ {stage_of.get(a)} vs {stage_of.get(b)}")
            else:
                reasons.append("same plan stage, plan ranks do not invert")
        return a, b, "; ".join(reasons)

    plan_only = [describe(p, in_plan=True) for p in sorted(plan_pairs - bound_pairs, key=sorted)]
    bound_only = [describe(p, in_plan=False) for p in sorted(bound_pairs - plan_pairs, key=sorted)]
    return PlanComparison(
        event_count=event_count,
        stage_count=stage_count,
        common=len(plan_pairs & bound_pairs),
        plan_only=plan_only,
        bound_only=bound_only,
    )


# --------------------------------------------------------------------------
# realised numbers from the paper's table sources
# --------------------------------------------------------------------------


def load_sources() -> dict[str, Any]:
    if not TABLE_SOURCES.exists():
        return {}
    return json.loads(TABLE_SOURCES.read_text(encoding="utf-8")).get("sources", {})


def paper_cell_for(benchmark: str, sources: dict[str, Any]) -> str | None:
    if benchmark in sources:
        return benchmark
    for cell, entry in sources.items():
        if entry.get("ours_benchmark") == benchmark:
            return cell
    return None


def realised_crossings(cell: str | None, sources: dict[str, Any]) -> dict[str, int | None]:
    if cell is None:
        return dict.fromkeys(PAPER_CONFIGS)
    entry = sources[cell]
    return {config: (entry.get(config) or {}).get("crossing_count") for config in PAPER_CONFIGS}


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------


def print_benchmark_report(
    result: BoundResult,
    comparison: PlanComparison | None,
    *,
    per_net: bool,
    examples: int,
) -> None:
    print(f"\n## {result.benchmark}")
    print(
        f"{result.instance_count} instances, {result.net_count} nets, "
        f"{len(result.columns)} columns, {len(result.bands)} bands with nets, "
        f"layout {result.layout_s:.1f} s"
    )
    print(f"lower bound (band-confined alternation count): {result.bound}")
    confined = sum(len(band.nets) for band in result.bands.values())
    print(f"nets inside the band model: {confined}; outside: {result.outside_count}")
    for reason, names in sorted(result.outside.items()):
        shown = ", ".join(f"{name} [{result.outside_detail[name]}]" for name in names[:examples])
        more = "" if len(names) <= examples else f", ... (+{len(names) - examples})"
        print(f"  - {reason}: {len(names)}: {shown}{more}")
    if result.recessed_terminals:
        print(
            f"terminals recessed from their wall face: {result.recessed_terminals} "
            f"(max {result.max_recess_um:.1f} um; ordered by y as if on the wall)"
        )
        print(
            "  worst: "
            + ", ".join(
                f"{net} at {where} ({recess:.1f} um)"
                for net, where, recess in result.worst_recess[:examples]
            )
        )
    ties = sum(band.tie_pairs for band in result.bands.values())
    if ties:
        print(f"pairs with equal y on a wall (not counted): {ties}")
    print(
        "per band: "
        + ", ".join(
            f"b{band.index}:{band.bound}/{len(band.nets)}n"
            for band in sorted(result.bands.values(), key=lambda b: b.index)
        )
    )
    if per_net:
        counts = result.per_net
        histogram = Counter(
            counts.get(net, 0)
            for band in result.bands.values()
            for net in (n.net for n in band.nets)
        )
        print(
            "per-net unavoidable crossings (count: nets): "
            + ", ".join(f"{k}: {v}" for k, v in sorted(histogram.items()))
        )
        top = counts.most_common(examples)
        print("  most constrained nets: " + ", ".join(f"{n}={c}" for n, c in top))
    if comparison is not None:
        if comparison.error:
            print(f"derived plan: not available ({comparison.error})")
        else:
            verdict = (
                "equal to the bound"
                if not comparison.plan_only and not comparison.bound_only
                else "differs from the bound"
            )
            print(
                f"derived plan: {comparison.event_count} events in {comparison.stage_count} "
                f"stages; {comparison.common} pairs shared, {len(comparison.plan_only)} "
                f"plan-only, {len(comparison.bound_only)} bound-only -> {verdict}"
            )
            for label, rows in (
                ("plan-only", comparison.plan_only),
                ("bound-only", comparison.bound_only),
            ):
                for a, b, reason in rows[:examples]:
                    print(f"  {label}: {a} x {b}: {reason}")
                if len(rows) > examples:
                    print(f"  {label}: ... (+{len(rows) - examples})")


def summary_row(
    result: BoundResult | None,
    benchmark: str,
    cell: str | None,
    realised: dict[str, int | None],
    comparison: PlanComparison | None,
    note: str | None = None,
) -> str:
    def fmt(value: int | None) -> str:
        return "--" if value is None else str(value)

    def gap(value: int | None) -> str:
        if value is None or result is None:
            return "--"
        return f"{value - result.bound:+d}"

    bound = "n/a" if result is None else str(result.bound)
    outside = "n/a" if result is None else str(result.outside_count)
    plan = "--"
    if comparison is not None and not comparison.error:
        plan = str(comparison.event_count)
        if comparison.plan_only or comparison.bound_only:
            plan += "*"
    cells = [cell or "--", benchmark, bound]
    for config in PAPER_CONFIGS:
        cells.append(f"{fmt(realised[config])} ({gap(realised[config])})")
    cells.extend([outside, plan, note or ""])
    return "| " + " | ".join(cells) + " |"


SUMMARY_HEADER = (
    "| cell | benchmark | bound | lidar (gap) | lidar-pure (gap) | contribution1 (gap) "
    "| contribution2 (gap) | nets outside model | derived plan events | note |\n"
    "|---|---|---:|---:|---:|---:|---:|---:|---:|---|"
)


def to_json(result: BoundResult, comparison: PlanComparison | None) -> dict[str, Any]:
    return {
        "benchmark": result.benchmark,
        "bound": result.bound,
        "instance_count": result.instance_count,
        "net_count": result.net_count,
        "layout_s": result.layout_s,
        "columns": [
            {"index": c.index, "xmin": c.xmin, "xmax": c.xmax, "instances": c.instances}
            for c in result.columns
        ],
        "bands": {
            str(band.index): {
                "bound": band.bound,
                "nets": [n.net for n in band.nets],
                "alternating_pairs": band.alternating_pairs,
            }
            for band in sorted(result.bands.values(), key=lambda b: b.index)
        },
        "per_net_bound": dict(sorted(result.per_net.items())),
        "outside": dict(sorted(result.outside.items())),
        "outside_detail": result.outside_detail,
        "derived_plan": None
        if comparison is None
        else {
            "event_count": comparison.event_count,
            "stage_count": comparison.stage_count,
            "common_pairs": comparison.common,
            "plan_only": comparison.plan_only,
            "bound_only": comparison.bound_only,
            "error": comparison.error,
        },
    }


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("benchmarks", nargs="*", help="benchmark module names")
    parser.add_argument(
        "--all-paper-cells",
        action="store_true",
        help="append every paper cell's `ours_benchmark` from docs/date2027_table_sources.json",
    )
    parser.add_argument("--per-net", action="store_true", help="print per-net bounds")
    parser.add_argument("--no-plan", action="store_true", help="skip the derived-plan comparison")
    parser.add_argument("--examples", type=int, default=5, help="examples listed per reason")
    parser.add_argument("--time-limit-s", type=float, default=600.0, help="per-benchmark layout")
    parser.add_argument("--json", type=Path, default=None, help="write details to this file")
    args = parser.parse_args(argv)

    sources = load_sources()
    benchmarks = list(args.benchmarks)
    if args.all_paper_cells:
        benchmarks.extend(
            entry.get("ours_benchmark", cell)
            for cell, entry in sources.items()
            if entry.get("ours_benchmark", cell) not in benchmarks
        )
    if not benchmarks:
        parser.error("name at least one benchmark or pass --all-paper-cells")

    rows: list[str] = []
    details: list[dict[str, Any]] = []
    for benchmark in benchmarks:
        cell = paper_cell_for(benchmark, sources)
        realised = realised_crossings(cell, sources)
        try:
            schematic, layout, layout_s = build_placed_layout(
                benchmark, time_limit_s=args.time_limit_s
            )
        except LayoutTimeout:
            note = f"layout exceeded {args.time_limit_s:.0f} s, not computed"
            print(f"\n## {benchmark}\n{note}")
            rows.append(summary_row(None, benchmark, cell, realised, None, note))
            continue
        boxes, links = extract_geometry(schematic, layout)
        result = compute_bound(benchmark, boxes, links, layout_s=layout_s)
        comparison = None if args.no_plan else compare_with_plan(benchmark, schematic, result)
        print_benchmark_report(result, comparison, per_net=args.per_net, examples=args.examples)
        rows.append(summary_row(result, benchmark, cell, realised, comparison))
        details.append(to_json(result, comparison))

    print("\n## summary (realised (realised - bound); * = derived plan differs from the bound)\n")
    print(SUMMARY_HEADER)
    print("\n".join(rows))
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(details, indent=1), encoding="utf-8")
        print(f"\ndetails written to {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
