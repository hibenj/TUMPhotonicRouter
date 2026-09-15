"""The mesh slice generator keeps a contiguous column range with unchanged
placements and only the nets whose both ends stay (owner idea 2026-09-15)."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "make_mesh_slice_benchmark", ROOT / "scripts" / "make_mesh_slice_benchmark.py"
)
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def _parent():
    return yaml.load(
        (ROOT / "benchmarks" / "data" / "multiportmmi_8x8.yml").read_text(), Loader=yaml.UnsafeLoader
    )


def test_columns_are_ordered_by_x_and_named_by_prefix():
    order = mod.columns_in_x_order(_parent()["schematic_placements"])
    assert order[0] == "gc1" and order[1] == "fanout_yb" and order[-1] == "gc_array_out_gc"


def test_slice_keeps_only_inner_nets_and_unchanged_placements():
    parent = _parent()
    order = mod.columns_in_x_order(parent["schematic_placements"])
    first, last = order[2], order[4]
    out = mod.slice_yaml(parent, first, last)
    kept = set(out["instances"])
    assert all(mod.column_of(n) in set(order[2:5]) for n in kept)
    for name, endpoints in out["nets"].items():
        a, b = (e.split(",")[0] for e in endpoints)
        assert a in kept and b in kept
        assert parent["nets"][name] == endpoints
    for name, placement in out["schematic_placements"].items():
        assert placement == parent["schematic_placements"][name]
    dropped = {n for n, (a, b) in parent["nets"].items() if (a.split(",")[0] in kept) != (b.split(",")[0] in kept)}
    assert dropped and not (dropped & set(out["nets"]))


def test_slice_rejects_reversed_range():
    parent = _parent()
    order = mod.columns_in_x_order(parent["schematic_placements"])
    try:
        mod.slice_yaml(parent, order[3], order[1])
    except ValueError:
        return
    raise AssertionError("reversed range must be rejected")
