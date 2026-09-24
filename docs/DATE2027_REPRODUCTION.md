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

## Verification (2026-09-22, commits bb317c9..bcc86e8, `results_date2027/`)

`reproduce_date2027.sh` in the order ladder, c2-mesh, mesh-base,
benes-large (sequential, 12:52-16:59, machine otherwise idle): all 27
cells of ours reproduced, every one exact on crossing count and GDS
length (to the archived 0.001 um), zero verifier errors, routing-loop time
ratio 0.92-1.14 to the archives. `compare_date2027.py` output:

| benchmark | config | crossings paper/repro | GDS length um paper/repro | errors | t_loop ratio | status |
|---|---|---|---|---|---|---|
| benes_4x4_flat | lidar-pure | 2 / 2 | 9484.511 / 9484.511 | 0 | 1.09 | ok |
| benes_4x4_flat | contribution1 | 2 / 2 | 9475.187 / 9475.187 | 0 | 1.06 | ok |
| benes_4x4_flat | contribution2 | 2 / 2 | 9475.255 / 9475.255 | 0 | 1.01 | ok |
| benes_8x8_flat | lidar-pure | 16 / 16 | 31418.224 / 31418.224 | 0 | 1.13 | ok |
| benes_8x8_flat | contribution1 | 16 / 16 | 31312.203 / 31312.203 | 0 | 1.06 | ok |
| benes_8x8_flat | contribution2 | 16 / 16 | 31348.89 / 31348.89 | 0 | 1.06 | ok |
| benes_16x16_flat | lidar-pure | 88 / 88 | 92323.991 / 92323.991 | 0 | 0.92 | ok |
| benes_16x16_flat | contribution1 | 88 / 88 | 92106.172 / 92106.172 | 0 | 1.02 | ok |
| benes_16x16_flat | contribution2 | 88 / 88 | 93654.793 / 93654.793 | 0 | 1.05 | ok |
| benes_32x32_flat | lidar-pure | 416 / 416 | 270827.61 / 270827.61 | 0 | 1.05 | ok |
| benes_32x32_flat | contribution1 | 416 / 416 | 270872.224 / 270872.224 | 0 | 1.04 | ok |
| benes_32x32_flat | contribution2 | 416 / 416 | 277886.703 / 277886.703 | 0 | 1.06 | ok |
| benes_64x64_flat | contribution2 | 1824 / 1824 | 1159313.33 / 1159313.33 | 0 | 1.04 | ok |
| benes_128x128_flat | contribution2 | 7680 / 7680 | 4938323.005 / 4938323.005 | 0 | 0.99 | ok |
| multiportmmi_8x8 | lidar-pure | 33 / 33 | 25160.335 / 25160.335 | 0 | 1.13 | ok |
| multiportmmi_8x8 | contribution1 | 33 / 33 | 25073.626 / 25073.626 | 0 | 1.07 | ok |
| multiportmmi_8x8 | contribution2 | 33 / 33 | 26319.97 / 26319.97 | 0 | 1.05 | ok |
| multiportmmi_16x16 | lidar-pure | 63 / 63 | 90200.066 / 90200.066 | 0 | 1.14 | ok |
| multiportmmi_16x16 | contribution1 | 63 / 63 | 89586.04 / 89586.04 | 0 | 1.04 | ok |
| multiportmmi_16x16 | contribution2 | 63 / 63 | 96331.277 / 96331.277 | 0 | 1.03 | ok |
| multiportmmi_32x32 | lidar-pure | 121 / 121 | 336038.575 / 336038.575 | 0 | 1.11 | ok |
| multiportmmi_32x32 | contribution1 | 121 / 121 | 334816.567 / 334816.567 | 0 | 1.01 | ok |
| multiportmmi_32x32 | contribution2 | 121 / 121 | 360764.408 / 360764.408 | 0 | 1.01 | ok |
| multiportmmi_64x64 | lidar-pure | 217 / 217 | 1305628.208 / 1305628.208 | 0 | 1.07 | ok |
| multiportmmi_64x64 | contribution1 | 209 / 209 | 1302769.96 / 1302769.96 | 0 | 1.06 | ok |
| multiportmmi_64x64 | contribution2 | 209 / 209 | 1408033.109 / 1408033.109 | 0 | 0.97 | ok |
| multiportmmi_128x128 | contribution2 | 311 / 311 | 5573562.981 / 5573562.981 | 0 | 1.08 | ok |

all reproduced cells match

Notes: the five ADEPT contribution 2 rows were archived with Milestone 9
in the kernel and reproduce exactly without it, so Milestone 9 never
influenced the paper's rows and stays absent from this branch. Benes
128x128 with contribution 2: loop 912 s (archive 925 s), wall 5443 s of
which the verifier is most (20480 routes). LiDAR rows are not covered.

## Re-verification on the restructuring branch

- 2026-09-23, commit c57bf31 (Milestone 1 of `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`: every environment variable became a typed configuration field): all 27 cells exact on crossings and GDS length, 0 verifier errors, loop-time ratio 0.88-1.05 (`results_m1_check/`, 10:49-14:38).
- 2026-09-23, commit 235be3f (Milestone 3: one `NetSearch` interface, the kernel split into `src/search/astar/`, the loop rewritten as `kernel::run`, the Dijkstra oracle engine): all 27 cells exact, 0 verifier errors, loop-time ratio 0.90-1.05 (`results_m3_check/`, 19:04-22:56).
- 2026-09-24, commit 59cd76f (Milestone 4: the negotiated loop as 241 lines over named policies, loop-level tests): all 27 cells exact, 0 verifier errors, loop-time ratio 0.88-1.04 (`results_m4_check/`, 23:59-03:50).
- 2026-09-24, commit f8a711b (Milestone 5: the Python session as typed stages, `route_benchmark`/`route_schematic`, `python -m photonic_router route`): all 27 cells exact, 0 verifier errors, loop-time ratio 0.90-1.05 (`results_m5_check/`, 06:39-10:34).
