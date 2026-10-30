"""The unavoidable crossings of a paper benchmark, from its placement alone.

Usage:
    crossing_lower_bound.py <benchmark> [<benchmark> ...] [--per-net]
    crossing_lower_bound.py --all-paper-cells

No routing runs: the benchmark's placed layout is built as the flow builds
it and `photonic_router.crossings.crossing_demand.build_crossing_demand` counts the
invariant crossing pairs of its bands (experiment 5e, 2026-10-29). The
number is what a "realised minus unavoidable" column of
`compare_date2027.py` or `summarize_run.py` subtracts; this module is the
helper those scripts import (`lower_bound_for_benchmark`), the wiring into
the table is a later slice.
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

TABLE_SOURCES = REPO_ROOT / "docs" / "date2027_table_sources.json"


@dataclass(frozen=True)
class LowerBound:
    benchmark: str
    bound: int
    per_net: dict[str, int]  # net name -> unavoidable crossings (nets with none absent)
    band_count: int
    outside_count: int  # nets outside the band model (0 on every paper benchmark)

    def gap(self, realised: int | None) -> int | None:
        """Realised minus unavoidable, or None when nothing was realised."""
        return None if realised is None else int(realised) - self.bound


def lower_bound_for_benchmark(benchmark: str) -> LowerBound:
    """Build the benchmark's placed layout and return its crossing lower bound."""
    from gdsfactory.gpdk import get_generic_pdk
    from photonic_router.crossings.crossing_demand import build_crossing_demand
    from photonic_router.records.layout_from_schematic import layout_from_schematic

    get_generic_pdk().activate()
    module = importlib.import_module(f"benchmarks.{benchmark}")
    if hasattr(module, "register_benes_cells"):
        module.register_benes_cells()
    schematic = module.build_schematic()
    demand = build_crossing_demand(schematic, layout_from_schematic(schematic))
    return LowerBound(
        benchmark=benchmark,
        bound=demand.lower_bound(),
        per_net=demand.counts_by_net_name(),
        band_count=len(demand.bands),
        outside_count=len(demand.outside),
    )


def paper_benchmarks() -> list[str]:
    """The `ours_benchmark` of every paper cell, in table order."""
    if not TABLE_SOURCES.exists():
        return []
    sources = json.loads(TABLE_SOURCES.read_text(encoding="utf-8")).get("sources", {})
    names: list[str] = []
    for cell, entry in sources.items():
        name = entry.get("ours_benchmark", cell)
        if name not in names:
            names.append(name)
    return names


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("benchmarks", nargs="*", help="benchmark module names")
    parser.add_argument(
        "--all-paper-cells",
        action="store_true",
        help="append every paper cell's benchmark from docs/date2027_table_sources.json",
    )
    parser.add_argument("--per-net", action="store_true", help="print the per-net bounds")
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        help="also write {benchmark: {bound, bands, outside, per_net}} to this file "
        "(docs/date2027_crossing_bounds.json is what compare_date2027.py --with-bounds reads)",
    )
    args = parser.parse_args(argv)
    benchmarks = list(args.benchmarks)
    if args.all_paper_cells:
        benchmarks.extend(name for name in paper_benchmarks() if name not in benchmarks)
    if not benchmarks:
        parser.error("name at least one benchmark or pass --all-paper-cells")

    print("| benchmark | unavoidable crossings | bands | nets outside the band model |")
    print("|---|---:|---:|---:|")
    table: dict[str, dict[str, object]] = {}
    for benchmark in benchmarks:
        bound = lower_bound_for_benchmark(benchmark)
        print(f"| {benchmark} | {bound.bound} | {bound.band_count} | {bound.outside_count} |")
        if args.per_net:
            print("  per net: " + ", ".join(f"{n}={c}" for n, c in bound.per_net.items()))
        table[benchmark] = {
            "bound": bound.bound,
            "bands": bound.band_count,
            "outside": bound.outside_count,
            "per_net": dict(sorted(bound.per_net.items())),
        }
    if args.json is not None:
        args.json.write_text(json.dumps(table, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        print(f"\nwritten {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
