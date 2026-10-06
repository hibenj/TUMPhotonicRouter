#!/usr/bin/env python
"""Extract one of the owner's private circuits into a benchmark module.

    .venv/bin/python scripts/extract_private_benchmark.py <name> [--report-only]

``<name>`` is a key of ``benchmarks/private/configs/circuits.py::CIRCUITS``
(git-ignored: it names confidential files and foundry cells). Writes
``benchmarks/private/<name>.py``, routable as
``python -m photonic_router route private.<name>``, and prints the extraction
report (components by kind, nets, stubs, nets to account for). Private-circuits
plan, 2026-10-05.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from translation.gds_circuit_extraction import extract_circuit  # noqa: E402
from translation.gds_circuit_rebuild import (  # noqa: E402
    default_generic_mappings,
    generic_schematic,
    write_benchmark_module,
)

STABLE_ROUTING_FLAGS: tuple[str, ...] = (
    "--allow-45-degree-turns",
    "false",
    # 5 um, not the cascades' 10: every MZI arm of these circuits leaves its
    # MMI 20 um before the heater input (an S-bend of two 10 um quarter arcs
    # fits exactly, and the grid router then crossed its own path or left no
    # straight segment for the endpoint correction)
    "--bend-radius-um",
    "5",
    "--path-length-matching",
    "false",
    "--include-heater-obstacles",
    "true",
    "--electrical-routing",
    "false",
    "--waveguide-clearance-um",
    "3.0",
    "--crossings",
    "false",
    # the generic couplers are shorter than the foundry ones, and a loop from
    # one coupler of a column to another runs around the column's back; the
    # die gets room beyond the 20 um security margin on every side
    "--chip-add-x-um",
    "100",
    "--chip-add-y-um",
    "100",
    # the MMI input ports are 2 um apart (1.33 um in one of the circuits): on the
    # 2 um default grid both land in one cell for some grid phases and the
    # second net can never reach its target
    "--grid-size-um",
    "1.0",
    # shortest nets first within the topological layers (see the rebuild's
    # net order); one Clements mesh does not route in declaration order
    "--net-order",
    "topological-span",
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name")
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args(argv)
    from benchmarks.private.configs.circuits import CIRCUITS, FLAG_OVERRIDES, NET_ORDERS

    gds_path, config = CIRCUITS[args.name]
    circuit = extract_circuit(gds_path, config)
    print(circuit.report())
    if args.report_only:
        return 0
    flags = STABLE_ROUTING_FLAGS + tuple(FLAG_OVERRIDES.get(args.name, ()))
    # the arm pair is drawn with the benchmark's own routing bends
    bend_radius_um = float(flags[flags.index("--bend-radius-um") + 1])
    schematic = generic_schematic(
        circuit,
        default_generic_mappings(bend_radius_um=bend_radius_um),
        net_order=NET_ORDERS.get(args.name, "chord"),
    )
    target = REPO / "benchmarks" / "private" / f"{args.name}.py"
    write_benchmark_module(
        schematic,
        target,
        title=f"Private benchmark {args.name} (extracted from the owner's tape-out GDS)",
        # a circuit's own flags come last: argparse keeps the last value of a flag
        flags=flags,
    )
    print(
        f"wrote {target.relative_to(REPO)}: {len(schematic.netlist.instances)} instances, "
        f"{len(schematic.netlist.routes)} nets"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
