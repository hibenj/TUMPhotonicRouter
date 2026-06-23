# Pass 18A - Shared Port-to-Routing-Grid Access Audit

## Scope

This checkpoint is an architecture audit and benchmark/verification plan. It
does not implement port-access routing yet.

Decision: start with metal routing in Pass 18B, but shape the data model so
photonic routing can use the same explicit "physical port -> legal grid anchor"
concept later.

## Problem Statement

Both routing domains currently depend on grid quantization near ports:

- Photonic routing converts a physical gdsfactory port directly into a routed
  Rust `State(x, y, angle)`.
- Electrical routing opens a disk of legal terminal cells and later realizes a
  continuous adapter from the selected physical terminal contact to the routed
  grid tail.

The missing shared concept is an explicit pre-A* access result:

1. true physical port/contact point,
2. selected legal grid anchor cell/state,
3. continuous access centerline from the physical port to that anchor,
4. cells opened or reserved for the access,
5. length and verification metadata.

Without this, route starts are implicit side effects of grid rounding or local
open-cell selection. For photonic path-length matching this is especially
dangerous: two otherwise matched ports can select different neighboring cells
and introduce a deterministic sub-grid length skew, such as a 0.5 um mismatch
on a 0.5 um routing grid.

## Current Photonic Behavior

Photonic routing enters through `translation/route_rust.py::route_nets_rust`.

Relevant current flow:

- A static obstacle map is built once from the unrouted layout.
- `port_to_grid_state()` moves one grid cell outward from the physical port and
  floors that physical point into a grid cell.
- `_states_and_openings()` uses those source/target states as the A* endpoints
  and adds the endpoint cells plus port reservation cells to the opened set.
- Route length records store `route_obj.total_length_um`, which is the Rust
  routed primitive length between snapped grid states.

Important code locations:

- `translation/route_rust.py:496` - photonic routing entry point.
- `translation/route_rust.py:732` - direct port-to-grid-state conversion.
- `translation/route_rust.py:1112` - source/target state and opened-cell setup.
- `translation/route_rust_records.py:38` - recorded route length comes from the
  route object.

There is already Rust-side access machinery:

- `src/geometry_realization.rs:469` - `PortAccessConfig`.
- `src/geometry_realization.rs:485` - `PortAccess` model.
- `src/geometry_realization.rs:800` - `build_port_access()`.
- `src/geometry_realization.rs:2235` -
  `realize_route_polygon_with_port_access()`.
- `src/py_router.rs:1184` - PyO3 exposure for `build_port_access()`.

Audit finding: this Rust `PortAccess` path is not currently wired into
`route_nets_rust()`. It also lacks obstacle-aware anchor selection today: it
selects a geometric anchor but does not prove the access stub avoids current
layout obstacles, dynamic routes, or photonic bend/length constraints.

## Current Metal Behavior

Electrical routing enters through
`translation/electrical/route_electrical.py::route_electrical_heaters`.

Relevant current flow:

- `build_electrical_obstacle_map()` builds an electrical-only obstacle grid.
- For each terminal, it opens cells around the side-selected physical contact
  point for common-bus and individual routing.
- Common-bus routing starts from terminal open cells.
- Individual topology routing starts from terminal open cells.
- Detailed bundle routing turns topology paths into offset point paths.
- Metal realization calls `terminal_access_path()` to draw a continuous physical
  adapter from the selected terminal contact to the first useful route point.
- Verification already models terminal contacts and terminal adapters.

Important code locations:

- `translation/electrical/obstacle_extraction.py:24` - electrical obstacle map.
- `translation/electrical/obstacle_extraction.py:62` - terminal open-cell disks
  are generated around side-selected port centers.
- `translation/electrical/common_bus_router.py:315` - local trunk starts choose
  nearest terminal open cell.
- `translation/electrical/common_bus_router.py:495` - shortest path starts from
  sorted terminal open-cell candidates.
- `translation/electrical/individual_topology.py:56` - individual routing uses
  terminal open cells as source candidates.
- `translation/electrical/terminal_contacts.py:83` - realized physical terminal
  adapter.
- `translation/electrical/verification.py:50` - terminal adapter overlap is an
  intentional same-net overlap class.

Audit finding: metal already solves the continuous geometry side at realization
time, which makes it the right first domain. The missing piece is that the
access choice is not an explicit pre-routing object. Route-start cells are
selected by each router stage from a terminal-open disk, and the physical access
stub is reconstructed later from the chosen route tail.

## Shared Access Model

Introduce a shared access contract before changing either routing algorithm.
The exact module name can be decided in 18B, but the model should be domain
neutral enough to support both metal and photonic:

```python
from dataclasses import dataclass
from typing import Literal

@dataclass(frozen=True)
class RoutingPortAccess:
    domain: Literal["electrical", "photonic"]
    owner_id: str
    port_name: str
    port_point_um: tuple[float, float]
    port_orientation_deg: float | None
    anchor_cell: tuple[int, int]
    anchor_point_um: tuple[float, float]
    anchor_angle: int | None
    access_centerline_um: tuple[tuple[float, float], ...]
    opened_cells: frozenset[tuple[int, int]]
    reserved_cells: frozenset[tuple[int, int]]
    access_length_um: float
    reason: str = "selected"
```

Domain-specific wrappers can add stricter fields:

- metal: terminal id, access purpose (`common_bus` or `individual`), contact
  bbox, wire width, and adapter rectangles.
- photonic: Rust `State`, bend-radius policy, minimum straight, route-entry
  angle, and length-accounting policy.

Key invariant: the route search receives anchor states/cells, not true physical
ports. The realized route must then include both access centerlines and the
routed body, and verification must check the full combined geometry.

## Metal-First 18B Plan

Pass 18B should make the existing metal behavior explicit without changing the
overall topology strategy.

Recommended scope:

1. Add a metal access resolver near the electrical package boundary, likely
   `translation/electrical/port_access.py`.
2. Build one access record per terminal and purpose:
   - `common_bus`: seed from the terminal port facing `config.bus_side`.
   - `individual`: seed from the terminal port facing `config.pad_side`.
3. Select a deterministic legal anchor cell from the existing terminal-open
   cells. Preserve the current ordering first:
   - common-bus local trunk: bias toward the local target x.
   - common-bus BFS: bias by Manhattan distance from terminal center.
   - individual topology: bias by pad-side escape direction.
4. Store selected access records on `ElectricalObstacleMap` or on a new
   electrical route-planning payload.
5. Make route stages consume access anchor cells instead of independently
   re-selecting raw terminal-open cells where possible.
6. Update realization to use the selected access record where available, with
   `terminal_access_path()` retained as the metal geometry builder.
7. Update verification metrics with access-specific values:
   - access count,
   - max physical-port-to-anchor offset,
   - max adapter length,
   - blocked anchor count,
   - missing access contact count.

The first implementation should preserve current geometry as much as possible.
If metrics change, the debug output should explain whether the change came from
anchor selection or from realization.

## Metal Verification Plan

Use the existing electrical benchmark ladder:

1. `mmi_heater` for the narrow single-heater check.
2. `mmi_heater_8x4` for many terminal accesses without rip-up pressure.
3. `mmi_heater_8x4_ripup_reroute` for the current full guardrail suite.

Required checks for 18B:

- `pytest tests/test_electrical_routing.py -q`
- `pytest tests/test_benchmark_electrical_script.py -q`
- `python scripts/benchmark_electrical.py mmi_heater --artifacts-dir build/18b_metal_access`
- `python scripts/benchmark_electrical.py mmi_heater_8x4_ripup_reroute --artifacts-dir build/18b_metal_access`

Guardrails to preserve:

- verification success is true,
- no failed detailed routes,
- no cross-net metal overlap,
- no blocked-cell clearance errors,
- same-net redundant overcount remains zero,
- pre-union metal overcount remains explained by intentional contact/adapter
  joins,
- metal snapshot SVG still shows terminal contacts, access adapters, route
  tails, and pads.

New unit tests should cover:

- off-grid physical electrical port resolves to an in-bounds legal anchor cell,
- blocked candidate cells are skipped,
- selected access centerline starts at the physical port and ends at the anchor
  center,
- both top and bottom pad-side access choose side-appropriate terminal ports,
- realization uses a provided access record instead of recomputing a different
  terminal contact from the route tail.

## Photonic Follow-Up Constraints

Photonic should not be the first implementation pass, but the metal model must
not make photonic harder.

Photonic-specific requirements before enabling access anchors:

- Access selection must be orientation aware and must produce a legal
  `State(anchor_x, anchor_y, anchor_angle)`.
- Access stubs must obey minimum straight and bend-radius constraints. A
  Manhattan-style adapter is not generally valid for waveguides.
- Access length must be included in `RoutedNetRecord.total_length_um` or in a
  clearly separate length term consumed by path-length matching.
- Matching groups need deterministic access pairing. Candidate selection should
  be able to minimize differential access length between related ports, not
  only choose each port independently.
- Debug records should expose source access length, target access length, A*
  body length, and total physical length.
- The realization path should use Rust
  `realize_route_polygon_with_port_access()` only after the routed primitive
  endpoints exactly match the selected access anchors.

For the known 0.5 um mismatch issue, the important design point is that snapping
cannot be hidden inside `port_to_grid_state()`. The resolver must either choose
symmetry-preserving anchor cells for all ports in a matching group, or it must
account for the induced access-length difference before matching acceptance is
evaluated.

## Recommended Pass Split

- 18B: Metal access records, metrics, debug exposure, and benchmark guardrails.
- 18C: Clean up metal route-stage consumers so all terminal start choices flow
  through access records.
- 18D: Photonic access resolver design spike with path-length accounting tests,
  no routing-flow default enablement.
- 18E: Photonic opt-in integration for one small benchmark and one synthetic
  off-grid matched-port case.
