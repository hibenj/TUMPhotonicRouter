# Crossing Baselines

This file tracks the crossing-routing baselines and experiment branches so the
LiDAR-style variants do not get conflated.

## Modes

| Mode | Branch | Crossing permission | A* behavior | Status |
| --- | --- | --- | --- | --- |
| `window` | `crossing-clean-baseline` | Precomputed topology crossing constraints | Opens expected crossing windows before search | Existing baseline |
| `collision` | `baseline/lidar-topology-crossings` | Precomputed topology crossing constraints | Routes normally, then legalizes collisions with expected/allowed committed routes | Implemented |
| `lidar-pure` | `baseline/lidar-pure-crossings` | Dynamic DRC-style checks, not precomputed topology pairs | Routes normally, then legalizes collisions with any committed route that passes local crossing checks | Implemented on this branch |

## Current Baseline: `collision`

`collision` is a LiDAR-style expansion baseline with topology-constrained
crossing permission. It does not open crossing windows. It still uses the
precomputed crossing topology as the permission and ordering table for which
committed routes may be crossed.

On a blocked neighbor, A* tries to legalize the move as a crossing. The move is
accepted only when the crossed pair is allowed by the topology constraints, the
orientation is valid, the straight-margin rule holds, the physical crossing
reservation is clear, and the final realized crossing validator accepts the
route. Crossing moves add `crossing_loss` to the A* cost.

## Current Branch: `lidar-pure`

`lidar-pure` matches LiDAR more directly than `collision`:

- Do not use precomputed crossing pairs as a permission table.
- On collision, inspect the actual committed route owner.
- Allow the crossing when crossings are enabled, orientations are valid, the
  crossing footprint fits, and unrelated geometry is absent from the footprint.
- Require straight access on both route arms equal to
  `crossing_half_size_cells + min_straight_cells_per_crossing + bend_runout_cells`.
- Add `crossing_loss` to A* cost for crossing moves.
- Track actual crossing events and crossing counts.
- Report insertion loss diagnostics as:

```text
insertion_loss =
    length_um * propagation_loss
  + bend_count_or_angle * bend_loss
  + crossing_count * crossing_loss
  + device_loss
```

The current implementation reports the route-dependent terms. `device_loss`,
explicit per-net crossing budgets, and nonzero propagation/bend loss defaults
are left as follow-up knobs.

## Benchmark Snapshot

All runs used `--crossings true`; `benes_16x16` runs used
`--foreign-port-keepout-cells 6`.

| Benchmark | Mode | Result | Total time | Expanded states |
| --- | --- | --- | ---: | ---: |
| `benes_8x8` | `window` | pass | 0.942 s | 3,094 |
| `benes_8x8` | `collision` | pass | 1.224 s | 3,098 |
| `benes_8x8` | `lidar-pure` | pass | 29.351 s | 23,361 |
| `benes_16x16 --debug-stop-after-route 31` | `window` | pass | 23.298 s | 90,782 |
| `benes_16x16 --debug-stop-after-route 31` | `collision` | pass | 27.259 s | 87,491 |
| `benes_16x16 --debug-stop-after-route 105` | `window` | pass | 45.931 s | 97,631 |
| `benes_16x16 --debug-stop-after-route 105` | `collision` | pass | 51.644 s | 94,340 |
| full `benes_16x16` | `window` | pass | 118.780 s | 143,989 |
| full `benes_16x16` | `collision` | pass | 138.592 s | 140,696 |

## Notes

- `collision` expands slightly fewer states on the `benes_16x16` cases, but its
  wall time is higher because each candidate crossing move runs more legality
  checks.
- `lidar-pure` now passes `benes_8x8` after using the same additive straight
  access rule in A* and realized-geometry validation. Without the topology
  permission/order table it still does more crossing legality work than
  `collision`.
- The untracked file `docs/photonic_router_graph_crossing_plm_full.tex` is an
  intentional local document and should not be included in these commits.
