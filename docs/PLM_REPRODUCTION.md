# Path-length matching: baseline and reproduction gate (phase 6 slice 0)

Phase 6 of the do-over touches path-length matching (PLM). Before any of it
is changed, this baseline pins what the PLM flow produces today on the
benchmarks that exercise it, the way the 27-cell DATE 2027 reproduction
(`docs/DATE2027_REPRODUCTION.md`) pins the crossing engine. Nothing in the
production code changed for this slice.

## What the baseline contains

Thirteen benchmarks, one run each, config label `plm`: `heater_s_mod` and
the twelve cascades `cascade_{8,12,16}_{compact,diagonal,folded,folded_compact}`.
Each is routed with its own `STABLE_ROUTING_FLAGS` (the cascades re-export
`heater_s_mod`'s block), which `routing_flow.py <bench>` applies by itself:
`--allow-45-degree-turns false --bend-radius-um 10 --path-length-matching
true --path-length-match-outputs true --include-heater-obstacles true
--electrical-routing true --waveguide-clearance-um 3.0 --crossings false`.

`docs/plm_baseline.json` holds, per benchmark, the quantities the gate pins:

| quantity | source | compared |
|---|---|---|
| `rc` | run.txt | exact |
| `photonic_error_count` | the photonic verification JSON (metrics.json) | exact |
| `electrical_error_count` | `electrical_routing.verification_error_count` in the GDS meta info | exact |
| `nets` | routed record count of the photonic verifier | exact |
| `gds_length_um` | total waveguide length on the routed GDS, WG layer area / 0.5 um (`summarize_run.py`; the meanders are part of the strips) | 0.01 um |
| `meander_inserted_um` | `meander_insertion_report.total_inserted_extra_length_um` (GDS meta info; the SUMMARY line prints it to 0.1 um) | 0.01 um |
| `plm_groups`, `plm_over_tolerance` | the flow's `SUMMARY` line | exact |
| `group_spread_um` | max over the matching groups of `realized_arrival_spread_um` (max minus min realized arrival of the group's incoming edges, `path_length_analysis.matching_group_diagnostics`) | 0.01 um |
| `output_spread_um` | the same for the output-coupler group (`output_matching_diagnostics`) | 0.01 um |
| `routing_loop_s`, `plm_phase_s`, `wall_s` | `net routing phase` and `path-length matching phase` lines, run.txt | information only |

The PLM result needs no extra archive step: gdsfactory writes the routed
layout's `info` (`path_length_analysis`, `meander_insertion_report`,
`meander_requirements`, `electrical_routing`, `photonic_verification`) into
the GDS as the top cell's `kfactory:info` meta info, and `plm_table.py`
reads it back with klayout. The e2e test
`tests/e2e/test_path_length_realized_gds.py` pins separately that these
realized lengths agree with the strips in the GDS net by net.

## Capture

Commit `6f1a46d` (branch `do-over/phase0-hygiene`), 2026-10-29, cda-server-4
(32 cores, worktree `/tmp/hien/photonic/TUMPhotonicRouter-final`, kernel built
16:07), all thirteen benchmarks at once through `parallel -j 13`; the whole
set took 24 s wall (the longest run, `cascade_16_diagonal`, 23 s). The
workstation reference run of `heater_s_mod` (6 s wall, routing loop 2.2 s
against 1.8 s on the server) reproduces the server's cell exactly, so the
baseline is machine-independent on the pinned quantities.

| benchmark | rc | errors ph/el | nets | GDS length um | meander um | groups | over tol | group spread um | output spread um | t_loop s | t_plm s | wall s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heater_s_mod | 0 | 0/0 | 81 | 68839.215 | 23582.107 | 26 | 0 | 0.0000 | 0.0000 | 1.8 | 0.2 | 5 |
| cascade_8_compact | 0 | 0/0 | 81 | 56025.116 | 6304.830 | 26 | 0 | 0.0000 | 0.0000 | 2.0 | 0.2 | 5 |
| cascade_8_diagonal | 0 | 0/0 | 81 | 65190.465 | 6344.447 | 26 | 0 | 0.0000 | 0.0000 | 2.5 | 0.2 | 6 |
| cascade_8_folded | 0 | 0/0 | 81 | 68839.222 | 23536.447 | 26 | 0 | 0.0000 | 0.0000 | 1.8 | 0.2 | 5 |
| cascade_8_folded_compact | 0 | 0/0 | 81 | 66439.222 | 23536.447 | 26 | 0 | 0.0000 | 0.0000 | 1.7 | 0.2 | 5 |
| cascade_12_compact | 0 | 0/0 | 129 | 120375.664 | 15550.543 | 42 | 0 | 0.0000 | 0.0000 | 4.7 | 0.5 | 9 |
| cascade_12_diagonal | 0 | 0/0 | 129 | 152705.493 | 15926.182 | 42 | 0 | 0.0000 | 0.0000 | 7.8 | 0.4 | 12 |
| cascade_12_folded | 0 | 0/0 | 129 | 163651.201 | 69958.182 | 42 | 0 | 0.0000 | 0.0000 | 4.2 | 0.4 | 9 |
| cascade_12_folded_compact | 0 | 0/0 | 129 | 152851.202 | 69958.182 | 42 | 0 | 0.0000 | 0.0000 | 3.5 | 0.5 | 8 |
| cascade_16_compact | 0 | 0/0 | 177 | 212471.658 | 29021.448 | 58 | 0 | 0.0000 | 0.0000 | 11.0 | 0.6 | 18 |
| cascade_16_diagonal | 0 | 0/0 | 177 | 280353.835 | 29961.024 | 58 | 0 | 0.0000 | 0.0000 | 16.1 | 0.6 | 23 |
| cascade_16_folded | 0 | 0/0 | 177 | 302243.930 | 140481.024 | 58 | 0 | 0.0000 | 0.0000 | 7.3 | 0.7 | 14 |
| cascade_16_folded_compact | 0 | 0/0 | 177 | 278243.931 | 140481.024 | 58 | 0 | 0.0000 | 0.0000 | 6.3 | 0.6 | 13 |

All thirteen route clean: rc 0, no photonic or electrical verifier error,
every group within tolerance, realized spread 0 to the 4th decimal (the
acceptance summary's `max_physical_residual_um` is 9e-13, floating-point
noise). There is no `clements_8x8`-style residual in this set. The folded
cascades spend two to five times the meander length of the compact ones
(`cascade_16_folded`: 140 481 um of 302 244 um is meander), which is the
cost phase 6 is about; the baseline records it, it does not judge it.

## Re-run and compare

```
scripts/results/plm_baseline.sh                  # all thirteen, sequential, into results_plm/
scripts/results/plm_baseline.sh cascade_16_folded  # one benchmark
parallel -j 13 scripts/results/plm_baseline.sh ::: $(scripts/results/plm_baseline.sh --list)
.venv/bin/python scripts/results/plm_table.py results_plm                 # the table
.venv/bin/python scripts/results/plm_table.py results_plm --compare docs/plm_baseline.json
.venv/bin/python scripts/results/plm_table.py results_plm --json docs/plm_baseline.json   # re-pin (owner decision)
scripts/results/plm_gate.sh                      # the short gate, about 10 s
```

`plm_baseline.sh` archives through `run_and_archive.sh` under `RESULTS_ROOT`
(default `results_plm/`, git-ignored, never `results/`), one hour timeout per
run (`PLM_TIMEOUT_S`), log `reproduce.txt`, and `touch results_plm/STOP`
skips the runs not yet started. A failed run (PLM acceptance raises before
the GDS is written) keeps its rc and the `Path-length matching failed: ...`
line in the table's note column; its other cells stay empty, and the compare
reports it as a mismatch against a clean baseline.

`plm_gate.sh` routes `heater_s_mod` and `cascade_8_compact` into a temporary
root and compares them with `--only-present`; it exits 0 only when both
cells match (`all PLM cells match`). It is the check for every later phase 6
slice; a slice that changes a pinned quantity on purpose re-runs the full
baseline and re-pins `docs/plm_baseline.json` in the same commit, with the
reason in the CHANGELOG.
