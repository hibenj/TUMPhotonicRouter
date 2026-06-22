# Profiling

This project now has two complementary profiling entry points.

## End-to-End Photonic Flow

Use this when measuring the full Python/Rust routing pipeline, including benchmark
loading, schematic translation, obstacle extraction, Rust routing, and realization.

```bash
.venv/bin/python scripts/benchmark_photonic.py --include-heater-obstacles --ripup-reroute
```

Useful variants:

```bash
.venv/bin/python scripts/benchmark_photonic.py mmi_heater_8x4_ripup_reroute --include-heater-obstacles --ripup-reroute
.venv/bin/python scripts/benchmark_photonic.py mmi_heater_8x4_ripup_reroute --path-length-matching --include-heater-obstacles --ripup-reroute
.venv/bin/python scripts/benchmark_photonic.py mmi_heater_8x4_ripup_reroute --include-heater-obstacles --ripup-reroute --attempt-output build/profiles/photonic_attempts.csv
.venv/bin/python scripts/benchmark_photonic.py mmi_heater_8x4_ripup_reroute --include-heater-obstacles --ripup-reroute --use-indexed-heap
.venv/bin/python scripts/benchmark_photonic.py --output docs/photonic_baseline.md --include-heater-obstacles --ripup-reroute
```

The `mmi_heater_8x4_ripup_reroute` case is the current end-to-end A* stress
case: it forces one rip-up repair and runs real A* searches in the photonic
pipeline. Use the default 5,000,000 iteration budget for this benchmark; lower
budgets such as 500,000 can fail before the repair route completes.

Use `--attempt-output` to write per-route-attempt records. A `.csv` suffix
writes a spreadsheet-friendly table; other suffixes write JSON. Each record
includes the attempt bucket (`normal_route`, `probe_route`,
`repair_failed_net`, or `reroute_victims`), net name, elapsed time, failure
state, repair round, expanded states, generated neighbors, heap push/pop
counts, duplicate heap skips, obstacle/clearance checks, routing-window
bounds/area, heap stale-skip split, maximum heap size, dense search-state
count, cost/parent update counts, primitive-class generated/accepted/rejected
breakdowns, footprint rectangle checks, dense-grid build time, and fallback
flags. The Markdown report also shows the slowest non-simple attempts inline.

The Python routing flow defaults to `--routing-window-scale 0.05` after the
window-diagnostic pass. On `mmi_heater_8x4_ripup_reroute`, this reduced route
42's largest window from about 806k cells to about 339k cells while preserving
the same repair count and successful route. Use `--routing-window-scale 0.35`
to reproduce the older, wider-window behavior.

Use `--use-indexed-heap` to benchmark the experimental decrease-key indexed
heap for dense A*. This mode keeps a single heap entry per state instead of
pushing duplicate entries for every improved cost. A successful queue
optimization run should reduce stale-generation heap skips and maximum heap
size while preserving route success and route cost.

## Isolated Rust A*

Use this when measuring only synthetic Rust A* routing scenarios through the PyO3
router API. This avoids gdsfactory and schematic/layout work.

```bash
.venv/bin/python scripts/profile_astar.py
```

Useful variants:

```bash
.venv/bin/python scripts/profile_astar.py straight_astar wall_gap_astar --iterations 100 --warmup 10
.venv/bin/python scripts/profile_astar.py --output build/profiles/astar.md --json-output build/profiles/astar.json
.venv/bin/python scripts/profile_astar.py --iterations 5 --warmup 1 --check-baseline
.venv/bin/python scripts/profile_astar.py wall_gap_astar slalom_astar --use-indexed-heap --iterations 25 --warmup 3
.venv/bin/python scripts/profile_astar.py --paired-comparison --iterations 25 --warmup 3
```

The A* profile reports median and p95 wall time plus Rust route counters such as
expanded states, generated neighbors, heap pushes/pops, skipped duplicate heap
entries, stale-generation versus already-closed heap skips, maximum heap size,
dense search-state count, cost/parent update counts, obstacle/clearance checks,
primitive-class generated/accepted breakdowns, footprint checks, dense-grid build time, neighbor-generation time,
heap-operation time, legality-check time, reconstruction time, full-grid
fallback use, target-state match, route cells, and route length.

The isolated profiler enables detailed Rust timing buckets explicitly. Normal
routing keeps detailed timing disabled by default so production runs still
collect counters without paying per-operation timing overhead.

The end-to-end photonic benchmark also collects quiet A* route-search counters
through `RoutingFlowStats`. Its Markdown report includes route attempts,
simple-route count, repair count, expanded states, generated neighbors, heap
push/pop counts, duplicate heap skips, stale-skip split, maximum heap size,
primitive-class mixes for slow attempts, obstacle/clearance checks, footprint
rectangle checks, full-grid fallbacks, and
aggregate A* search time. The worker JSON rows additionally include load/layout
time and the Rust timing buckets for neighbor generation, heap operations,
legality checks, and reconstruction.

Primitive-class counters use four buckets: short straight (`s`), long straight
(`l`), 45-degree bend (`b45`), and 90-degree bend (`b90`). Use the generated,
accepted, and footprint-rejected mixes to decide whether a later pass should
reorder primitives, add early filters, or remove dominated primitive variants.

Use `--paired-comparison` to run selected scenarios twice: once with baseline
A* and once with the accelerator flag requested. When no scenario names are
provided, paired mode defaults to synthetic `jps4_*` scenarios that use a
plain 4-connected grid baseline and the JPS4 unit-grid accelerator mode. The
paired report shows time, expansion, generated-neighbor, heap-operation, route
length, target-cell, and fallback deltas side by side.

The baseline check compares the current route length and reached target state
against `docs/astar_quality_baseline.json`. Use it before and after heuristic
or cost-function edits so speedups do not silently lengthen routes or break
exact port-orientation cases. You can temporarily override the accepted length
slack with `--length-tolerance-um`.

The profiler includes a generated 4x4 two-object orientation matrix:

```bash
.venv/bin/python scripts/profile_astar.py object_ports_n_n object_ports_n_e object_ports_n_s object_ports_n_w \
  object_ports_e_n object_ports_e_e object_ports_e_s object_ports_e_w \
  object_ports_s_n object_ports_s_e object_ports_s_s object_ports_s_w \
  object_ports_w_n object_ports_w_e object_ports_w_s object_ports_w_w
```

Each `object_ports_<source>_<target>` case places two rectangular obstacles side
by side and routes between N/E/S/W ports. The source starts with the port's
outward orientation; the target requires the opposite angle so the route
approaches into the target port. These cases are useful for detecting cost-model
or heuristic bias across port orientations.

Use the isolated A* profile for pass 5 search-kernel optimization. Use the
end-to-end photonic profile to check that lower-level improvements survive the
real routing pipeline.
