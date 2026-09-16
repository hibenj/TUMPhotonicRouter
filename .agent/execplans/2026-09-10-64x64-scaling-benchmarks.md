# 64x64 scaling benchmarks: Benes and multiport MMI under LiDAR, baseline, contribution 1 and contribution 2

## Purpose

The paper's evaluation stops at 32 inputs. The owner wants one size step beyond that, so that the scaling argument has a data point where the loss-driven routers run into their limits and the pre-placed crossing structures (contribution 2) still solve the circuit. After this plan, the repository has two new benchmarks, `benes_64x64` and `multiportmmi_64x64`, each routable with the same one-flag commands as the smaller sizes, and a recorded result for four configurations per benchmark: the LiDAR reference router, our engine in its baseline `lidar-pure` mode, contribution 1 (`--crossing-mode lidar-guided`) and contribution 2 (`--preplaced-crossing-grids true`), all with a one-hour limit. The owner's expectation, to be confirmed or refuted by the runs: the two loss-driven routers do not finish within the hour, contribution 2 does.

A "benchmark" here is a Python module in `benchmarks/` that builds the circuit (a gdsfactory schematic: components, their placement, and the nets between their ports) and declares the stable routing flags of its baseline. A "planned crossing" is a crossing the topology analysis derives from the port orders before routing; "verification-clean" means the photonic verification JSON in `build/verification/` reports zero errors.

## Decision log

- 2026-09-10 (owner): generate both 64x64 benchmarks and add them to the suite; run LiDAR, lidar-pure, contribution 1 and contribution 2 with a one-hour limit each; the hope is that at least contribution 2 solves them.
- 2026-09-10: the multiport MMI 64x64 is generated with LiDAR's own generator (`src/picroute/benchmarks/MMIports.py`, `generate_netlist(64, die_area=[26000, 12800], seed=1234)`), i.e. the die area doubled from the 32x32 call and the same seed. It is therefore a new random instance in LiDAR's format, not a published LiDAR benchmark; the paper must say so.
- 2026-09-10: Benes has no LiDAR counterpart (TUM-only topology). Running LiDAR on `benes_64x64` needs an exporter from our schematic to LiDAR's YAML; see Milestone 3. Until it exists, the LiDAR column for Benes is "no counterpart".

- 2026-09-11 16:30 (fix, contribution 2 on benes_64x64): the unroutable switch-to-cell segment was the router's diagonal halo rule, not the geometry. The last 45-degree step into a tile port checks the cell beside the landing cell in the step direction (`compact_diagonal_halo_cells` in `src/astar.rs`); for the failing tile that cell was part of the tile's rasterized footprint, because the tile centre sat mid-cell and the arm's 0.25 um half width spilled one cell past the port cell. The 32x32 tiles happened to sit at a sub-cell offset where the footprint starts at the port column. Fix in the structure builder (`translation/preplaced_crossing_grids.py`: `_router_grid_origin`, `_port_cells_stay_inside_footprint`, `_snap_x_tile_center`): every X-array tile centre is snapped to a routing-grid cell boundary in x (a free parameter of the structure), and in y only when the port-cell condition fails at the row midpoint (at most half a cell). Result: benes_64x64 routes clean under contribution 2 -- 4416 routes, 0 failures, 0 repairs, 1824 tiles, both verifications 0 errors, 93 s wall (A* 50 s). Grid ladder afterwards unchanged: benes 4/8/16/32 and multiportmmi 8/16/32/64 all 0 failures (mm64 one repaired attempt as before), 0 errors. Unit test `test_x_array_tiles_snap_to_the_routing_grid_so_port_cells_are_footprint_edge_cells`; full pytest 399 passed, same 10 baseline failures.
- 2026-09-10 18:45: 1100 um is not enough either: all 1824 tiles are placed, but the router cannot connect two consecutive levels of the outer layer whose lane runs 100 um vertically between them (`No route found for n_s0_0_o1_to_s1_16_i0__via2`, an 8 um horizontal gap for two 45-degree bends). Rule adopted: the same room per swap level as the 32x32 has (29.6 um at 15 levels) -- 31 levels need a 1500 um stage pitch (30.5 um per level). `STAGE_PITCH_UM = 1500.0`. Follow-up for the selector: its 16 um level threshold is too optimistic when lanes run far vertically between levels.
- 2026-09-10 18:30: `benes_64x64` used a stage pitch of 1100 um (generator default 1000 um). Reason: its outer shuffle layers have 31 swap levels; at 1000 um the band between two stages leaves 14.3 um per level, below the 14.7 um a crossing cell with its 45-degree arms needs, so the selector sent both layers (128 nets) to the guided router and the first contribution 2 run did not finish in 290 s. At 1100 um every level has 17.5 um. The placement is an input of all four configurations, so the baseline and contribution 2 runs that had started with 1000 um were stopped and restarted.

## Milestone 1: the two benchmarks exist and load (done 2026-09-10)

`benchmarks/benes_64x64.py` is the `benes_32x32.py` parameter block with `NETWORK_SIZE = 64` and the same stable flags (`--crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs --foreign-port-keepout-cells 0 --proactive-congestion-weight 4.0 --proactive-congestion-radius-cells 3 --max-iterations 20000000`, env `PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT=1.0`). The generic generator `benchmarks/benes.py` accepts any power of two. Size: 13 stages, 480 switch nodes, 1824 planned crossings (32x32: 416).

`benchmarks/multiportmmi_64x64.py` is the `multiportmmi_32x32.py` block with `N = 64` reading `benchmarks/data/multiportmmi_64x64.yml` (copied from the LiDAR checkout `~/Documents/Repositories/working/LiDAR/src/picroute/benchmarks/multiportmmi_64x64/`). Size: 639 instances, 1218 nets in the YAML (32x32: 346 / 638).

Check: `PYTHONPATH=. .venv/bin/python -c "from benchmarks import benes_64x64, multiportmmi_64x64"` imports both; the routing flow finds a benchmark by module name (`routing_flow.py` imports `benchmarks.<name>`), so `python routing_flow.py benes_64x64` and `python routing_flow.py multiportmmi_64x64` are the baseline commands.

## Milestone 2: the four runs per benchmark, one hour each

Commands (from the repository root, `.venv` active, `timeout 3600` in front of each):

- LiDAR (from `~/Documents/Repositories/working/LiDAR/src/picroute`, its own `.venv`): `python main/picroute.py --benchmark=benchmarks/multiportmmi_64x64/multiportmmi_64x64.yml --config=config/comp_LiDAR.yml --run.output_layout_gds_path=result/LiDAR/scaling/multiportmmi_64x64_comp_LiDAR.gds`, log in `log/LiDAR/scaling/`.
- Baseline: `python routing_flow.py <benchmark>` (stable flags = lidar-pure).
- Contribution 1: `python routing_flow.py <benchmark> --crossing-mode lidar-guided` (hybrid net order is its default).
- Contribution 2: `python routing_flow.py <benchmark> --preplaced-crossing-grids true`.

Recorded per run: finished or timed out, wall time, attempts/failures/repairs, realized crossings vs planned, verification result, and for contribution 2 the tiles and the structure per layer. Runs are sequential or at most two in parallel on the 16-core machine so that wall times stay comparable. The runs start only on the owner's word (2026-09-10: plan first; go given 18:00).

Results table (filled in as runs finish):

| benchmark | LiDAR | baseline lidar-pure | contribution 1 | contribution 2 |
|---|---|---|---|---|
| multiportmmi_64x64 | **timeout 3600 s** in rip-up iteration 1 of 10 (13:48) | **timeout 3600 s** (no result; progress not logged in the non-verbose run) | **timeout 3600 s** at route 795 of 895, 0 failures until then | not started | **clean: 1853 routes, 1856 attempts / 1 failure / 1 repair, 209 tiles on 5 layers, verification 0 errors, 116 s wall** |
| benes_64x64 | **timeout 3600 s** in rip-up iteration 1 of 10 (12:48) | **timeout 3600 s** at the 1500 um pitch (no result; progress not logged) | **timeout 3600 s** at route 670 of 768 (in the outer layer 10->11), 0 failures until then | not started | **clean at the 1500 um pitch after the tile snap fix: 4416 routes, 0 failures, 0 repairs, 1824 tiles, verification 0 errors, 93 s wall** (history: 1000 um sent the two outer layers to the guided router; 1100 um left an 8 um gap for two bends between levels; 1500 um exposed the halo-rule parity issue, see the decision log) |

Memory (owner decision 2026-09-11): the machine has 30 GB with other applications holding about 9 GB; LiDAR needed 11 GB on the 64x64 MMI mesh after 25 min. A watchdog stops the largest routing process when less than 1.5 GB remain available and logs it (`memory_watchdog.log` in the session scratchpad). A run stopped this way is recorded as "out of memory" in the table, which counts as a failure of that configuration on that benchmark, the same as the one-hour timeout.

## Milestone 3: Benes in LiDAR's format (done 2026-09-11, owner: "we need these numbers")

LiDAR has no Benes benchmark in any of its checkouts or upstream (the LiDAR 2.0 paper evaluates Benes 16x16 / 32x32 built from MZI switches after Qiao et al. 2017, but neither those benchmarks nor the LiDAR 2.0 code are public). The bridge is two scripts:

- `scripts/lidar_bridge/export_benes_spec.py <benchmark> <spec.json>` (our venv): builds our unrouted layout and writes every instance with the lower-left corner of its bounding box and its LEF orientation (`N`, or `FN` for the mirrored input couplers), every switch expanded into its primitives `<switch>__mmi_in`, `__mmi_out`, `__heater_top`, `__heater_bottom` (the switch's internal waveguides are left to LiDAR), and every net: our routed nets with switch ports mapped to the primitives, plus the four switch-internal connections per switch. Insertion losses per component as in LiDAR's own generators (coupler 2.0, MMI 0.3, heater 0.1 dB), placement halo 10 um, die = layout bounding box + 100 um. The gdsfactory components `mmi2x2`, `straight_heater_metal`, `grating_coupler_te` have bit-identical bounding boxes and ports in LiDAR's gdsfactory 8.26 and our 9.43 (checked), so lower-left anchoring reproduces our placement exactly.
- `scripts/lidar_bridge/benes_from_spec.py <spec.json>` (LiDAR's venv, run from `src/picroute/benchmarks`): builds the YAML with LiDAR's own `CustomSchematic` writer (macro library, `FIXED` placements, `layout.yml`), so the file is exactly what LiDAR's generators produce. Output `src/picroute/benchmarks/benes_<n>x<n>/benes_<n>x<n>.yml`.

Sizes: benes_8x8 96 instances / 128 nets (48 ours + 80 internal); 16x16 256 / 352; 32x32 640 / 896; 64x64 1536 / 2176 (the 64x64 with the 1500 um stage pitch). Note for the paper: LiDAR's net count includes the switch-internal nets our flow does not route (they are inside the switch cell); LiDAR's per-net crossing count lists every crossing on both nets, so its total is twice the physical count.

Validation on benes_8x8: LiDAR routes it in 21 s, 128 nets, DRV 0 on every net, 32 per-net crossings = 16 physical crossings = exactly our plan, total wirelength 32.5 mm (with the internal nets). Command: `python main/picroute.py --benchmark=benchmarks/benes_8x8/benes_8x8.yml --config=config/comp_LiDAR.yml --run.output_layout_gds_path=result/LiDAR/benes/benes_8x8_comp_LiDAR.gds`, log `log/LiDAR/benes/benes_8x8.log`.

LiDAR on the Benes ladder (bridge benchmarks, `config/comp_LiDAR.yml`, 10 rip-up iterations max):

| benchmark | LiDAR wall | nets | DRV | physical crossings (LiDAR per-net count / 2) | our plan | LiDAR wirelength incl. internal nets |
|---|---|---|---|---|---|---|
| benes_8x8 | 21 s | 128 | 0 | 16 | 16 | 32.5 mm |
| benes_16x16 | 1363 s (5 rip-up iterations) | 352 | 0 | 88 | 88 | 94.4 mm |
| benes_32x32 | **timeout 3600 s** in rip-up iteration 1 of 10 (our baseline: 997 s wall, clean) | 896 | - | - | 416 | - |
| benes_64x64 | **timeout 3600 s** in rip-up iteration 1 of 10 | 2176 | - | - | 1824 | - |

Launched 2026-09-11 10:25 as one sequential chain (1 h each): LiDAR benes_16x16, benes_32x32, benes_64x64, multiportmmi_64x64 (logs `log/LiDAR/benes/` and `log/LiDAR/scaling/`); in parallel our baseline on benes_64x64 and multiportmmi_64x64 (1 h each).

## Follow-ups decided 2026-09-11 (owner)

1. Fix contribution 2 on benes_64x64 -- done 2026-09-11 (tile snap, see the decision log).
2. Contribution 1 probably completes with a larger limit: at the one-hour timeout it had 0 failures and stood at route 670 of 768 (Benes) and 795 of 895 (mesh). Reruns with a larger limit (e.g. 3 h, same for the baseline and LiDAR so the comparison stays fair) are to be scheduled by the owner; not started.

## Result summary (2026-09-11 13:50)

All runs of Milestone 2 are done. The memory watchdog never had to intervene. On both 64x64 benchmarks the three loss-driven configurations time out at one hour: LiDAR (in its first rip-up iteration, on the MMI mesh with 11 GB), our baseline, and contribution 1 (0 failures up to the timeout, benes at route 670 of 768, the mesh at 795 of 895). Contribution 2 routes the 64x64 MMI mesh clean in 116 s and, after the tile snap fix of 2026-09-11, the 64x64 Benes network clean in 93 s (4416 routes, 1824 tiles, 0 failures). LiDAR through the bridge on the Benes ladder: 8x8 and 16x16 clean with exactly the planned crossings, 32x32 and 64x64 time out.

## Acceptance

Both benchmarks import, both route under contribution 2 verification-clean within the hour (the expectation), and the table above is complete with the four columns per benchmark, each entry either a clean result with its wall time or "timeout 3600 s at net N of M".

## Milestone 3 (owner go 2026-09-13 21:40): 4 h limit at 64x64, contribution 2 alone at 128x128

Owner decisions: (1) rerun the three loss-driven configurations on both 64x64 benchmarks with a 4 h limit (contribution 1 stood at route 670 of 768 / 795 of 895 with 0 failures at the 1 h timeout, so a finish is plausible); a run the memory watchdog stops is labelled "out of memory" and counts as failed. (2) 128x128: only contribution 2 is run; the loss-driven configurations and LiDAR are recorded as failed without a run (they time out already at 64x64).

Runner: session scratchpad `overnight64/` (`launch.sh`, `run_one.sh`, `watchdog.sh`, `followup.sh`, `chain128.sh`; per run `<name>.log`, `<name>.status` with rc / wall / peak RSS / label, `memory_watchdog.log`). Labels: clean, timeout 14400 s, out of memory (watchdog), failed rc=N. Our runs use `--verbose-routes` so a timeout still records "route N of M".

Schedule (sequential lanes, two of our runs in parallel on the same benchmark so both see the same load): lane A: lidar-pure + lidar-guided on benes_64x64 (started 21:42), then on multiportmmi_64x64. LiDAR runs alone afterwards (multiportmmi_64x64, then benes_64x64), then contribution 2 on benes_128x128 and multiportmmi_128x128 (4 h each).

Incident 21:45: LiDAR on multiportmmi_64x64 was started alongside our two runs; the kernel OOM killer stopped it after 172 s at 8.4 GB RSS (desktop applications held about 17 GB, our two runs 2.5 GB each, and the first watchdog version forked itself into 800 subshells). That attempt is invalid (shared load), not a LiDAR failure; both LiDAR runs are rescheduled to run alone after lane A. The watchdog was rewritten without recursion (one `ps` snapshot per 5 s).

New benchmarks for milestone 3: `benchmarks/benes_128x128.py` (15 stages, 1088 nodes, 1792 nets, 7680 planned crossings; stage pitch 2500 um by the 64x64 rule: 63 swap levels x 29.6 um + 556 um) and `benchmarks/multiportmmi_128x128.py` with `benchmarks/data/multiportmmi_128x128.yml` (LiDAR's generator, `generate_netlist(128, die_area=[52000, 25600], seed=1234)`, 1179 instances, 1791 nets; a new random instance, not a LiDAR benchmark). Both import and build their schematic. Results: to be filled in when the chain finishes.

Incident 22:46 (contribution 1, benes_64x64, 4 h run): after all 768 nets had been dispatched (64 min, 0 failures) the kernel panicked in `CrossingHookContext::build` (`src/astar.rs:3975`, "dense grid should build for a valid obstacle map"): the lidar-mode crossing hook rasterizes the full routing bounds, and the 64x64 die (8380 x 3439 = 28.8M cells) exceeds `AStarConfig.max_dense_obstacle_cells` = 10M, so the builder returns `None` and the `expect` fires. The 32x32 dies stay under the cap; the mesh 64x64 (83M cells) would fail the same way. Fix 23:20 (`translation/route_rust.py`, `dense_obstacle_cell_cap`): the cap is sized from the obstacle map (2 x width x height, never below the default), set on the config right after it is created; unit test in `tests/test_route_rust_obstacle_config.py`; benes_8x8 lidar-guided re-run bit-identical (190 890 expanded, 48/0/0, 16 crossings, 0 errors). Lane A was restarted with the fix at 23:25 (`laneA2.sh`); the lidar-pure benes_64x64 run in progress (1.5 h, at route 671) was stopped, since it would have hit the same cap. Logs of the pre-fix attempts: `overnight64/before-capfix/`.

Result 2026-09-14 00:18 (contribution 1, benes_64x64, 4 h limit, cap fix in): **failed after 3860 s** -- no panic any more; all 768 nets dispatched with 0 search failures, then the source-layer repair could not restore or reroute `n_s6_2_o1_to_s7_1_i1` (stage 6 -> 7, source (4864, 3228), target (5384, 3338)) and the flow raised `No route found` (`translation/route_rust.py` `_dispatch_native_routing`). Peak RSS 5.45 GB. Table entry: failed (unroutable net in repair, 64 min). Log: `overnight64/ours_lidar-guided_benes_64x64.log`.

Result 2026-09-14 03:14 (baseline lidar-pure, benes_64x64, 4 h limit, cap fix in): **timeout 14400 s at route 671 of 768** (outer layer 10 -> 11), 0 search failures until then, peak RSS 6.17 GB. Table entry: timeout 4 h at route 671 of 768. Mesh pair (lidar-pure + lidar-guided on multiportmmi_64x64) started 03:14.

Result 2026-09-14 04:13 (baseline lidar-pure, multiportmmi_64x64, 4 h limit): **failed after 3539 s** -- all 895 nets dispatched, 0 search failures, then the source-layer repair could not restore or reroute nets 309-311 after a failed center-out attempt for net 313 and the flow raised `No route found` for `n_308` (mmi0_multiport_0_7,o18 -> mmi0_ps_array_1_heater_53,o1). Peak RSS 6.07 GB. Table entry: failed (unroutable nets in repair, 59 min). Contribution 1 on the mesh still running (at route 795 of 895 after 2 h).

Result 2026-09-14 06:39 (contribution 1 lidar-guided, multiportmmi_64x64, 4 h limit): **clean in 12 315 s wall** (A* 12 243 s): 1100 attempts / 63 failures / 42 repairs, 568 000 607 expanded, 275 realized crossings = 209 planned + 66 unplanned (vs 209 tiles under contribution 2), photonic and crossing verification 0 errors, peak RSS 4.47 GB. First loss-driven configuration to finish a 64x64 benchmark; 115x the contribution 2 wall time (107 s) with 66 more crossings. Lane A finished 06:39; LiDAR multiportmmi_64x64 started alone 06:40 (12.3 GB after 31 min, 9.7 GB still available).

Result 2026-09-14 08:09 (original LiDAR, multiportmmi_64x64, alone, 4 h limit): **clean in 5330 s wall** (rip-up iterations used: 4 of 10), 895 nets, DRV 0 on every net, 224 physical crossings (448 per-net entries; plan 209, contribution 2 = 209, contribution 1 = 275), total wirelength 1291.9 mm, peak RSS 14.08 GB (the 1 h attempt on 2026-09-11 timed out in iteration 1 under shared load; alone and with 4 h it finishes). Table entry: clean, 89 min, 224 crossings. LiDAR benes_64x64 started alone 08:09.

Owner decision 2026-09-14 08:53: chain stopped. LiDAR benes_64x64 killed 44 min in (rip-up iteration 1 of 10, 212 route decisions, 5 GB); table entry: stopped, no verdict. The 128x128 contribution 2 runs were not started (benchmarks `benes_128x128`, `multiportmmi_128x128` are in the repository, ready). Milestone 3 results table (4 h limit, our runs two in parallel, LiDAR alone): benes_64x64 -- LiDAR stopped 44 min, baseline timeout at route 671/768, contribution 1 failed in repair at 64 min, contribution 2 clean 93 s; multiportmmi_64x64 -- LiDAR clean 89 min / 224 crossings, baseline failed in repair at 59 min, contribution 1 clean 3 h 25 min / 275 crossings, contribution 2 clean 107 s / 209 crossings.

Result 2026-09-14 09:03 (original LiDAR, multiportmmi_16x16, alone, 4 h limit, owner go 08:58): **clean in 231 s**, 3 of 10 rip-up iterations, DRV 0, peak RSS 1.32 GB, 63 physical crossings = plan, wirelength 87.6 mm (our lidar-pure: 43 s wall, 33 s A*, 227/2/1, 65 crossings). LiDAR multiportmmi_32x32 started 09:02.

Result 2026-09-14 09:24 (original LiDAR, multiportmmi_32x32, alone, 4 h limit): **clean in 1330 s**, 3 of 10 rip-up iterations, DRV 0, 121 physical crossings = plan, wirelength 330.8 mm, peak RSS 3.96 GB (our lidar-pure: 307 s wall, 272 s A*, 457/0/5, 121 crossings, route length 334.7 mm). LiDAR mesh ladder is now complete: 8x8 65 s / 16x16 231 s / 32x32 1330 s / 64x64 5330 s; ours: 14 / 43 / 307 s / failed.

Result 2026-09-15 08:07 (original LiDAR, benes_64x64 via the bridge, alone, **12 h cap**, owner go 2026-09-14 19:45): **timeout 43 200 s**, reached rip-up iteration 2 of 10 at 11 h 53 min (iteration 1 alone took almost 12 h), peak RSS 11.58 GB, no result. Table entry: timeout 12 h. On this ladder LiDAR: 8x8 21 s, 16x16 23 min, 32x32 timeout 1 h, 64x64 timeout 12 h.

## Lane A/B results (2026-09-16 night, `scratchpad/overnight64/exp/`)

- Lane A (ours, negotiated engine, repo build of 2026-09-15 15:11 = Milestones 1-6 without the crossing-free rule): A1 benes_64x64 lidar-pure stopped by the owner at 2 h 40 min (round 2); A2 multiportmmi_64x64 lidar-guided clean 2268 s, 217 crossings; A3 benes_64x64 lidar-guided **timeout 4 h** (round 2 reached at 2 h 18 min, 49 nets redone by the cap, peak 5.75 GB); A4 **benes_128x128 contribution 2 clean, 715 s wall, A* 76 s, 17 152 routes, 0 failures, peak 5.37 GB**; A5 multiportmmi_128x128 contribution 2 **failed at 101 s**: `No route found for n_1643: mmi1_ps_array_1_heater_108,o2 -> mmi1_multiport_1_0,o109` (55th job, after the 54 crossing-tile nets). Diagnosis run with `PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG=1` (`scratchpad/night2/mm128_diag.log`): 99 `search-failure-blocker` lines, all `static=true`, in two clusters (x=15894-15898, y=2944-2953 and x=9153-9154, y=2650-2660, grid cells) -- the fan-in from the 128 heaters (x=15052) to the 128 MMI input ports (x=15992, 5 um pitch) is closed by static geometry, not by routed nets. To check in the morning: which instances own those cells (the 128-port MMI is 580 um wide and 2088 um long; the heater array's bodies; the static stubs), and whether the LiDAR generator's die of 52 x 25.6 mm leaves the fan-in room the 64x64 has.
- Lane B (original LiDAR at `loss_crossing: 300`, our 200 um price; `comp_LiDAR_x200.yml`): benes_8x8 clean 106 s, 1 iteration, 16 physical crossings, DRV 0; multiportmmi_8x8 clean 113 s, 3 iterations, 30 crossings, DRV 0; benes_16x16 clean 2730 s, 4 iterations, 88 crossings, DRV 0 (price 0 run: 1363 s); **multiportmmi_16x16 crashed after 560 s in rip-up iteration 4 with an `IndexError: index 2 is out of bounds for axis 0 with size 2` in LiDAR's own path post-processing (`x2, y2 = new_points[start_index + 1]`, a two-point route) -- a LiDAR bug triggered at this price, not a routing failure (`exp/B_multiportmmi_16x16_x200.log`)**; benes_32x32 started 02:54; then mm32, mm64 (4 h), benes64 (12 h).

