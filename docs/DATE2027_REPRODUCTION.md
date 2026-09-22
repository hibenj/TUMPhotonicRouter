# DATE 2027 results: engine, provenance and reproduction

Branch `DATE2027-results` holds the engine that produced every row of ours
in the paper's results table (`EXPERIMENTS_TABLE_SOURCES.json` next to the
paper's `main.tex`, written by `scripts/results/paper_table.py`). The paper
is final; this branch exists so the numbers can be regenerated.

## The engine on this branch

- Kernel: the frozen baseline engine, commit `8ddde83` (`src/py_router.rs`
  restored to that state), plus one addition: the long-straight congestion
  penalty exemption for dense fan-out nets (commit `a181115`,
  `set_long_straight_exempt_net_ids`, `long_straight_weight_override`).
  `git diff 8ddde83 -- src/py_router.rs` shows nothing else.
- Contribution 2 configuration: `--preplaced-crossing-grids true`.
  `routing_flow.py` then sets `PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT=1`
  (unless the shell sets it to `0`), and `translation/route_rust.py` hands the
  nets with a static fan-out anchor on a dense multi-port instance to the
  kernel, which searches them with `long_straight_congestion_weight = 0`.
  Benes benchmarks have no dense instances, so the exemption is a no-op there.
- Not on this branch: Milestone 9 of the negotiated engine (span-scaled first
  budgets, retry budget after a rip-up, clean-probe escalation; commit
  `39ea78d`). It regresses the Benes ladder and was never part of the
  baseline / contribution 1 rows. See "Verification" for the mesh
  contribution 2 rows, which were archived with it in the kernel.
- Baseline (`--crossing-mode lidar-pure`) and contribution 1
  (`--crossing-mode lidar-guided`) run the kernel with no extra environment.
- Build: `RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu .venv/bin/maturin develop --release`
  (`rust-toolchain.toml` pins a Windows channel; the Linux host needs the override).

## Provenance of the archived rows (results/)

Every archive's `run.txt` records `commit=`. `--verbose-routes` only adds
per-route log lines; all lengths in the table are measured on the routed GDS.

| rows | benchmark files | commit | worktree | arguments |
|---|---|---|---|---|
| Benes 4x4 .. 32x32, baseline | `benes_<n>x<n>_flat` | 8ddde83 | frozen | `--crossing-mode lidar-pure` |
| Benes 4x4 .. 32x32, contribution 1 | `benes_<n>x<n>_flat` | 8ddde83 | frozen | `--crossing-mode lidar-guided` |
| Benes 4x4 .. 128x128, contribution 2 | `benes_<n>x<n>_flat` | 8ddde83 | frozen | `--preplaced-crossing-grids true --verbose-routes` |
| ADEPT 8x8, 16x16, baseline | `multiportmmi_<n>x<n>` | 96735e9 (= 8ddde83 minus a docs line) | main | `--crossing-mode lidar-pure` |
| ADEPT 32x32, 64x64, baseline | `multiportmmi_<n>x<n>` | 8ddde83 | main | `--crossing-mode lidar-pure` |
| ADEPT 8x8 .. 64x64, contribution 1 | `multiportmmi_<n>x<n>` | 8ddde83 | main | `--crossing-mode lidar-guided` |
| ADEPT 8x8 .. 32x32, contribution 2 | `multiportmmi_<n>x<n>` | a181115 (8ddde83 + Milestone 9 + exemption) | main | `--preplaced-crossing-grids true --verbose-routes` |
| ADEPT 64x64, 128x128, contribution 2 | `multiportmmi_<n>x<n>` | 7965428 (same engine as a181115) | main | `--preplaced-crossing-grids true --verbose-routes` |

The frozen worktree (`TUMPhotonicRouter-frozen`, detached at 8ddde83) carried
the `benes_<n>x<n>_flat.py` benchmark files and the switch-expanding
`benchmarks/benes.py`, byte-identical to this branch's copies.

LiDAR rows (`lidar_p300`, `lidar_p300_12h`, `lidar_p0*` archives) come from
the reference router via `scripts/results/lidar_12h*.sh` and the bridge in
`scripts/lidar_bridge/`; they are not reproduced by the script below.

## Reproduction

```
scripts/results/reproduce_date2027.sh            # all rows of ours, sequential
scripts/results/reproduce_date2027.sh ladder     # Benes ladder only, all three configurations
scripts/results/reproduce_date2027.sh c2-mesh    # the five mesh contribution 2 rows
.venv/bin/python scripts/results/compare_date2027.py results_date2027 <path>/EXPERIMENTS_TABLE_SOURCES.json
```

Archives go to `results_date2027/` (`RESULTS_ROOT`), never to `results/`, so
the paper's archives stay the latest under their labels. The comparison is
exact on crossing count and GDS length and requires zero verifier errors;
routing-loop time is printed as a ratio and not compared, since it depends
on the machine. `paper_table.py results_date2027 <main.tex>` would regenerate
the table from a full reproduction (the LiDAR columns then need the LiDAR
archives copied or linked into that root).

## Verification

Pending. The plan: Benes ladder in all three configurations against the
frozen archives (expected exact, same engine), then the five mesh
contribution 2 rows against their archives. Those were produced with
Milestone 9 in the kernel; if crossings or lengths differ without it,
Milestone 9 goes behind the contribution 2 switch on this branch instead
of being absent.
