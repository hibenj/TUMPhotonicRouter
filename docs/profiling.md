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
.venv/bin/python scripts/benchmark_photonic.py mmi_heater_8x4 --include-heater-obstacles --ripup-reroute
.venv/bin/python scripts/benchmark_photonic.py mmi_heater_8x4 --path-length-matching --include-heater-obstacles --ripup-reroute
.venv/bin/python scripts/benchmark_photonic.py --output docs/photonic_baseline.md --include-heater-obstacles --ripup-reroute
```

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
.venv/bin/python scripts/profile_astar.py --paired-comparison --iterations 25 --warmup 3
```

The A* profile reports median and p95 wall time plus Rust route counters such as
expanded states, generated neighbors, heap pushes/pops, skipped duplicate heap
entries, obstacle/clearance checks, footprint checks, dense-grid build time,
neighbor-generation time, heap-operation time, legality-check time,
reconstruction time, full-grid fallback use, target-state match, route cells,
and route length.

The isolated profiler enables detailed Rust timing buckets explicitly. Normal
routing keeps detailed timing disabled by default so production runs still
collect counters without paying per-operation timing overhead.

The end-to-end photonic benchmark also collects quiet A* route-search counters
through `RoutingFlowStats`. Its Markdown report includes route attempts,
simple-route count, repair count, expanded states, generated neighbors, heap
push/pop counts, duplicate heap skips, obstacle/clearance checks, footprint
rectangle checks, full-grid fallbacks, and aggregate A* search time. The worker
JSON rows additionally include load/layout time and the Rust timing buckets for
neighbor generation, heap operations, legality checks, and reconstruction.

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
