"""Cut a self-contained slice out of a multiport-MMI mesh benchmark.

A generated mesh is a chain of component columns (fanout, MZI array, phase-shifter
array, multiport MMI, ...); the nets of one band connect two neighbouring
columns and nothing crosses a column. A slice that keeps a contiguous range
of columns with every placement coordinate unchanged is therefore a real
benchmark of its own: the same geometry, ports and crossings in that region,
and the routing order inside it is the parent's order restricted to the kept
nets (net names are kept, so `n_574` is the same net as in the parent).

Owner idea 2026-09-15 (ExecPlan 2026-09-14-lidar-style-negotiated-ripup-endgame,
"physically build the benchmark where only these bands are in"): reproduce an
endgame stall of the 64x64 mesh in minutes and let every engine, the ladder
scripts and the verification run on it unchanged.

Usage (repository root, venv):
    .venv/bin/python scripts/make_mesh_slice_benchmark.py multiportmmi_64x64 \
        --first-column mmi0_ps_array_2 --last-column mol_array_1 \
        --name multiportmmi_64x64_bands8to9
writes benchmarks/data/<name>.yml and benchmarks/<name>.py. Column names are
the instance-name prefixes printed by --list-columns.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

COLUMN_PATTERN = re.compile(
    r"^(gc1|gc_array_out_gc|fanout_yb|mol_array_\d+|mmi[01]_ps_array_\d+|mmi[01]_multiport_\d+)"
)


def column_of(instance_name: str) -> str:
    match = COLUMN_PATTERN.match(instance_name)
    if match is None:
        raise ValueError(f"instance {instance_name!r} does not belong to a known column")
    return match.group(1)


def columns_in_x_order(placements: dict) -> list[str]:
    xs: dict[str, float] = {}
    for name, placement in placements.items():
        column = column_of(name)
        xs[column] = min(xs.get(column, float("inf")), float(placement["x"]))
    return sorted(xs, key=xs.__getitem__)


def slice_yaml(data: dict, first_column: str, last_column: str) -> dict:
    order = columns_in_x_order(data["schematic_placements"])
    lo, hi = order.index(first_column), order.index(last_column)
    if lo > hi:
        raise ValueError("first column must lie left of the last column")
    kept_columns = set(order[lo : hi + 1])
    kept_instances = {n: v for n, v in data["instances"].items() if column_of(n) in kept_columns}
    kept_placements = {n: v for n, v in data["schematic_placements"].items() if n in kept_instances}
    kept_nets = {}
    for net_name, endpoints in data["nets"].items():
        a, b = (endpoint.split(",")[0] for endpoint in endpoints)
        if a in kept_instances and b in kept_instances:
            kept_nets[net_name] = endpoints
    out = dict(data)
    out["instances"] = kept_instances
    out["schematic_placements"] = kept_placements
    out["nets"] = kept_nets
    out["constraints"] = {
        k: v
        for k, v in (data.get("constraints") or {}).items()
        if all(obj in kept_instances for obj in v.get("objects", []))
    }
    return out


MODULE_TEMPLATE = '''"""{name}: slice of {parent} keeping the columns {first} .. {last}
({n_instances} instances, {n_nets} nets), generated {date} by
scripts/make_mesh_slice_benchmark.py. Same placements, ports and stable flags
as the parent; net names are the parent's, so the routing order is the
parent's order restricted to these nets."""

from __future__ import annotations

from pathlib import Path

from gdsfactory.schematic import Schematic

from benchmarks.multiportmmi_yaml import build_schematic_from_lidar_yaml
from benchmarks.{parent} import STABLE_ROUTING_ENV, STABLE_ROUTING_FLAGS  # noqa: F401

BENCHMARK_YAML = Path(__file__).with_name("data") / "{name}.yml"
N = {n_side}
NODE_TYPES: dict[str, str] = {{}}
INTERNAL_DELAYS_UM: dict[str, float] = {{}}


def build_schematic() -> Schematic:
    """Build the {name} schematic."""
    return build_schematic_from_lidar_yaml(
        BENCHMARK_YAML, node_types=NODE_TYPES, internal_delays_um=INTERNAL_DELAYS_UM
    )
'''


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("parent", help="parent benchmark module name, e.g. multiportmmi_64x64")
    parser.add_argument("--first-column")
    parser.add_argument("--last-column")
    parser.add_argument("--name")
    parser.add_argument("--list-columns", action="store_true")
    args = parser.parse_args()
    yaml_path = ROOT / "benchmarks" / "data" / f"{args.parent}.yml"
    data = yaml.load(yaml_path.read_text(), Loader=yaml.UnsafeLoader)
    if args.list_columns:
        for column in columns_in_x_order(data["schematic_placements"]):
            print(column)
        return 0
    if not (args.first_column and args.last_column and args.name):
        parser.error("--first-column, --last-column and --name are required")
    out = slice_yaml(data, args.first_column, args.last_column)
    out_yaml = ROOT / "benchmarks" / "data" / f"{args.name}.yml"
    out_yaml.write_text(yaml.dump(out, Dumper=yaml.Dumper, sort_keys=False))
    import datetime as _dt
    import importlib

    n_side = getattr(importlib.import_module(f"benchmarks.{args.parent}"), "N", 0)
    (ROOT / "benchmarks" / f"{args.name}.py").write_text(
        MODULE_TEMPLATE.format(
            name=args.name, parent=args.parent, first=args.first_column, last=args.last_column,
            n_instances=len(out["instances"]), n_nets=len(out["nets"]),
            date=_dt.date.today().isoformat(), n_side=n_side,
        )
    )
    print(f"{args.name}: {len(out['instances'])} instances, {len(out['nets'])} nets -> {out_yaml.name}, benchmarks/{args.name}.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
