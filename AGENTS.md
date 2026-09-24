# AGENTS.md - TUMPhotonicRouter AI Agent Guide

## Agentic Coding Workflow

This file is the stable repository guide for agents. It should describe the
project architecture, common commands, and local conventions that are true
across many tasks.

The repository-level goal lives in `.agent/PROJECT_GOAL.md`. Task-specific
plans belong in `.agent/execplans/`, not in this file. Before creating or
revising a task plan, read `.agent/PLANS.md` and follow its ExecPlan format.
For multi-agent collaboration, role boundaries, handoff rules, and validation
expectations, read `.agent/WORKFLOW.md`. If acting as a lead agent that
delegates to subagents, also read `.agent/ORCHESTRATOR.md`.
For commits and repository state checkpoints, read `.agent/GIT_WORKFLOW.md` and
keep `.agent/REPOSITORY_STATE.md` current before every agent stop, pause, or
handoff.

Use this operating sequence for non-trivial work:

1. Orient: read this file, the relevant code, and any active ExecPlan.
2. Plan: create or update one self-contained ExecPlan in `.agent/execplans/`
   when the work spans multiple files, algorithms, or validation stages.
3. Implement: keep changes scoped to the plan and update the ExecPlan as
   discoveries or decisions occur.
4. Review: run a review pass focused on correctness, regressions, missing
   tests, and repository fit before considering the task complete.
5. Validate: run the narrowest meaningful checks first, then broader test or
   benchmark commands when the risk warrants it. Record evidence in the
   ExecPlan for large tasks.

Do not treat old task plans in `Agent_implementation_files/` or
`.agent/execplans/` as standing policy. They are historical or task-specific
artifacts unless the current user request explicitly resumes them.

## Project Overview

**TUMPhotonicRouter** is a hybrid Rust+Python photonic integrated circuit (PIC) routing system. It translates circuit schematics to unrouted layouts, then routes photonic waveguides using a grid-based A* algorithm with primitive-based path primitives (straights and angle-quantized bends).

**Tech Stack:** Rust (core routing engine) + Python (orchestration, gdsfactory integration), PyO3 bindings, Maturin build system.

---

## Critical Architecture Patterns

### Architecture: the flow and the two engines

**Read `docs/ARCHITECTURE.md` first.** It is the current module map (Rust and
Python, one paragraph per module), the literal stage sequence with the input and
output type of each phase, the search interface and the negotiation loop with
their policies, the configuration precedence, the build/test commands, and the
Milestone 8 removal candidates. `docs/CONFIGURATION.md` is the generated
reference for every configuration field. The summary:

```
python -m photonic_router route <benchmark> --configuration {baseline,contribution1,contribution2}
  photonic_router/cli.py      parse the command line, build RoutingConfig + FlowOptions
  photonic_router/flow.py     route_benchmark -> route_schematic: load, layout,
                              optional pre-placed crossing grids, optical routing,
                              verification, path-length matching, electrical, write
  translation/routing/        the routing session: nine phases behind the Protocols
                              of stages.py, driven by session.py::run
  src/search/                 one net: the NetSearch interface, AStarSearch, GridDijkstraSearch
  src/engine/                 many nets: commitment, repair, and negotiation/ (the loop
                              plus its budget / crossing-free / rip-up / queue policies)
  src/bindings/               the PyO3 surface: convert and delegate, never decide
```

**Key Files:**
- `docs/ARCHITECTURE.md` - **Start here**: the module map, the flow, the interfaces
- `python/photonic_router/flow.py` - `route_benchmark(config, options)` and `route_schematic`
- `python/photonic_router/cli.py` - the command line; `routing_flow.py` is a shim over it
- `translation/routing/session.py` - the nine-phase routing session
- `src/search/mod.rs`, `src/engine/negotiation/loop_.rs` - the two interfaces to read first

### Hybrid Language Boundary

The Python-Rust boundary is critical:
- **Python**: Schematic loading, GDS/gdsfactory manipulation, primitive placement, net extraction
- **Rust** (via PyO3): Grid obstacle maps, A* pathfinding, primitive library configuration, state management
- **Bindings**: `photonic_router._rust` module (built by maturin from `src/`)

**Build Pipeline:**
```bash
maturin develop  # Builds Rust → python/photonic_router/_rust.so
cargo build      # Alternative: build only Rust
```

On this Linux host the toolchain and `PYO3_PYTHON` have to be overridden; the
exact build and test commands are in `docs/ARCHITECTURE.md` section 8.

On this Windows workspace, the repo pins Rust to
`stable-x86_64-pc-windows-gnullvm` via `rust-toolchain.toml` because MSVC
`link.exe` is not installed. Use the project virtualenv and the repo-local
Cargo config:

```powershell
C:\Users\benja\.cargo\bin\cargo.exe check
.\.venv\Scripts\python.exe -m maturin develop --release
```

If Rust builds start failing with `link.exe not found`,
`x86_64-w64-mingw32-clang not found`, or `no Python 3.x interpreter found`, read
`docs/WINDOWS_RUST_TOOLCHAIN.md` before changing toolchains.

**Import Pattern** (critical):
```python
import photonic_router._rust as rust_backend  # Bindings from maturin

rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)
```

### Data Flow: Coordinates & State

**Grid Coordinate System:**
- Physical (µm) ↔ Grid (integer cells via `physical_to_grid()`)
- Grid cell `(x, y)` represents cell center in physical space
- Angles: 8-bit discrete (0-7 = 0°, 45°, 90°, ..., 315°)
- State tuple: `(x_grid, y_grid, angle_idx)` where angle_idx ∈ [0,7]

**Port Representation:**
- `gdsfactory.Port` has `.center` (physical) and `.orientation` (degrees, 0-360)
- Conversion: `_orientation_to_angle()` in `translation/routing/route_jobs.py` normalizes orientation → 0-7

### Primitive Library: 1:1 Mapping

**Key Insight**: Rust returns primitive IDs → Python looks up components in `PrimitiveLibrary`:
- 56 primitives total: 8 angles × (2 straight lengths + 4 bend angles)
- Each primitive has metadata: start/end angle, length, cost
- Placed directly via `layout.add_ref()` with rotation + translation

**Critical File:** `python/photonic_router/primitive_library.py`
- Singleton pattern: `get_primitive_library()` caches globally
- Bend factory fallback: prefers `bend_euler_all_angle()` for 45° bends

### Obstacle Map: Static vs Port-Open Cells

Built from placed instances in layout:
- **blocked_cells**: Geometry of shields/structures (cannot pass)
- **port_open_cells**: Temporary open zones around ports (routing can originate/terminate here)
- Built by `build_static_obstacle_map()` → calls Rust backend `build_static_obstacle_map_rs()`

---

## Developer Workflows

### Quick Start: Run Existing Benchmark

```bash
# From the repository root
.venv/bin/python -m photonic_router route benes_8x8_flat --configuration baseline
```

`--configuration {baseline,contribution1,contribution2}` selects one of the
three paper configurations (exactly one runs at a time); see
`docs/ARCHITECTURE.md`. The older script form still works and is what the
reproduction scripts use, because `routing_flow.py` is a shim over the same
command line:

```bash
.venv/bin/python routing_flow.py benes_8x8_flat --crossing-mode lidar-pure
```

Produces (with `--debug-svgs <selector>`):
- `build/static_obstacles/<benchmark>_obstacles.svg` - Grid obstacles
- `build/routes/<benchmark>_*.svg` - Per-net routing paths
- `build/routed_<benchmark>.gds` - The routed layout

### Add New Benchmark

1. Create `benchmarks/MY_DESIGN.py`:
```python
from gdsfactory.schematic import Schematic, Instance, Placement


def build_schematic() -> Schematic:
    schematic = Schematic()
    schematic.add_instance("comp1", Instance(component="grating_coupler_te"), Placement(x=0, y=0))
    schematic.add_instance("comp2", Instance(component="mmi_1x2"), Placement(x=100, y=0))
    # Add nets via schematic.netlist.routes (see benchmarks/heater_single.py for pattern)
    return schematic
```

2. Run: `.venv/bin/python -m photonic_router route MY_DESIGN --debug-svgs 1-5 --show-klayout`
   (`--debug-svgs` takes a 1-based route selector; `all` writes one SVG per net),
   or from a notebook
   `from photonic_router.flow import route_benchmark; route_benchmark(config, options)`
   with the two objects of `docs/CONFIGURATION.md`

3. Inspect in `build/` directory

### Test Rust Backend Build

```bash
maturin develop --release  # Build Rust extension with optimizations
pytest tests/test_rust_backend_import.py -v
```

Verifies: `PyPhotonicRouter`, `GridSpec`, `build_static_obstacle_map_rs` exposed.

### Debug Routing Failures

Common issues:
1. **Backend not available**: `RuntimeError: Rust router backend is not available`
   - Run `maturin develop` first
2. **Port extraction fails**: Check `get_port_from_instance()` → component must have instances named correctly
3. **Route fails**: Likely due to blocked cells trapping endpoints. Check SVG obstacles in `build/`.

### Adding Tests

Tests live in `tests/` (end-to-end ones in `tests/e2e/`):
- `tests/test_rust_backend_import.py` - Verify Rust bindings
- `tests/test_routing_stages.py` - One test per routing phase on a synthetic layout
- `tests/e2e/test_routing_flow_stats.py` - Full pipeline on a benchmark
- Pattern: take a fixture from `tests/fixtures/` (via `tests/conftest.py`) and use
  `pytest` with temp directories for artifacts

---

## Project-Specific Patterns

### 1. Dynamic Benchmark Loading

Benchmarks are **pure Python functions**, not YAML:
```python
# benchmarks/heater_single.py
def build_schematic() -> Schematic:
    # Return schematic - single source of truth
```

**Why**: Enables parametric designs, no external config, imports work naturally. Load via:
```python
benchmark_module = importlib.import_module(f"benchmarks.{name}")
schematic = benchmark_module.build_schematic()
```

### 2. Modular Translation Layer

`translation/` directory = interchangeable routing implementations:
- `layout_from_schematic.py` - Fixed (Schematic → unrouted Component)
- `route_gds.py` - Baseline gdsfactory router (reference implementation)
- `translation/routing/` - Production Rust router as nine phases; `session.py::run`
  is the sequence, `stages.py` declares one `Protocol` per phase, `api.py` is the
  boundary the flow calls (`translation/route_rust.py` only re-exports it)

**Interface contract**: `route_nets_rust(unrouted_layout, schematic, ...) -> (Component,
RustRouteDebugArtifacts)`, and per phase the Protocols of
`translation/routing/stages.py`.

Allows swapping a router, or one phase of it, without touching the other stages;
`docs/ARCHITECTURE.md` section 3 lists the phases with their types.

### 3. Grid Discretization & Clearance

Obstacles expanded by **security_margin** (default 20µm) + **clearance** (0.5µm) for routing clearance.

**GridSpec Invariants:**
- `width`, `height` in cells
- `grid_size_um` typically 0.5µm
- Origin at `(origin_x_um, origin_y_um)` in physical space

### 4. State Machine Routing

A* operates on **State** tuples: `(x, y, angle)` where angle is quantized to 8 octant directions.

**Primitives define transitions**: Each primitive specifies angle change (e.g., straight keeps angle, 45° bend rotates by ±1 octant).

### 5. Debug Artifacts Generation

Enable via `debug_dir` parameter:
```python
route_nets_rust(layout, schematic, debug_dir="build", debug_prefix="my_design")
# Outputs: build/static_obstacles, build/routes/*.svg
```

SVGs show:
- Grid with blocked/open cells
- Primitive sequences traced on grid
- Used for validation & visualization

---

## Critical Files Reference

Line counts are `wc -l` at 2026-09-24; `docs/ARCHITECTURE.md` has the full map.

| File | Lines | Purpose | When to Edit |
|------|-------|---------|--------------|
| `docs/ARCHITECTURE.md` | - | The module map and the interfaces | Orienting; after any structural change |
| `python/photonic_router/flow.py` | 809 | The flow (`route_benchmark`, `route_schematic`) | Adding or reordering a flow stage |
| `python/photonic_router/cli.py` | 811 | The command line and `--configuration` | Adding a flag |
| `python/photonic_router/config.py` | 412 | `RoutingConfig` / `RouterConfig` | Adding a configuration field |
| `translation/routing/session.py` | 372 | The nine-phase routing session | Changing the phase sequence |
| `translation/routing/stages.py` | 207 | One `Protocol` per phase | Changing a phase's contract |
| `translation/routing/state.py` | 339 | `SessionState`, documented per field | Adding per-run state |
| `python/photonic_router/primitive_library.py` | 179 | Component library | Adding primitives, bend config |
| `src/search/mod.rs` | 128 | The `NetSearch` interface | Adding a search engine |
| `src/search/astar/kernel.rs` | 1,479 | The A* loop | Algorithm tuning |
| `src/search/astar/cost.rs` | 546 | Every g-cost term | Cost/heuristic tuning |
| `src/engine/negotiation/loop_.rs` | 1,506 | The negotiated rip-up loop | Loop behaviour |
| `src/engine/negotiation/{budget,ripup,crossing_free,queue}.rs` | 960 | The loop's policies | Changing one rule |
| `src/bindings/mod.rs` | 3,441 | The PyO3 method surface | Exposing something to Python |
| `src/obstacle_map.rs` | 1,725 | Grid discretization | Cell packing, clearance metrics |
| `src/primitives.rs` | 427 | Primitive definitions | Routing moves, state transitions |
| `src/config.rs` | 307 | The kernel's typed configuration | Adding a kernel parameter |

---

## Integration Points & Dependencies

### External: gdsfactory
- Used for: Component definition, port data structures, layout manipulation
- **Pattern**: Import `from gdsfactory.component import Component` and `from gdsfactory.schematic import Schematic`
- Note: Port orientation in degrees; convert to grid angles via `_orientation_to_angle()`

### External: PyO3/Maturin
- **Role**: Rust↔Python bindings
- **Build**: `pyproject.toml` defines module name `photonic_router._rust`
- **Key Configs**: 
  - `python-source = "python"` - Rust extension built into `python/` directory
  - `crate-type = ["rlib", "cdylib"]` in `Cargo.toml`

### Internal: Obstacle Map ↔ A* Router Loop
1. Build static obstacle map from layout via Rust (preserves cleared port cells)
2. For each net, instantiate `PyPhotonicRouter` with same grid spec
3. Router marks routed cells as blocked for next net (rip-up & reroute ready)

---

## Key Configuration Parameters

**Every configuration field, with its default, its `PHOTONIC_ROUTER_*` overlay
name and its command-line flag: `docs/CONFIGURATION.md`** (generated from the
dataclasses; regenerate with
`.venv/bin/python scripts/generate_config_reference.py`). Precedence: defaults <
benchmark stable block < environment < command line.

**Built by `translation/routing/router_setup.py` (phase 2) for the kernel:**
- `GridSpec`: width/height (cells), grid_size_um, origin
- `PrimitiveLibraryConfig`: grid_size_um (must match GridSpec)
- `AStarConfig` (`src/search/astar/config.rs`): iteration caps, bend weight,
  target tolerance, window scale

**From `static_obstacle_builder.py`:**
- `grid_size_um`: 0.5µm (cell size)
- `security_margin_um`: 20µm (instance expansion)
- `clearance_um`: 0.5µm (routing clearance)
- `port_open_radius_um`: 0.5µm (port access zone)

Tuning these affects routing density, speed, and success rate.

---

## Common Debugging Steps

1. **Check SVG obstacles**: Is geometry correctly rasterized?
   - Look at `build/static_obstacles/*.svg`
   
2. **Trace port extraction**: Are port names correct?
   - Print from `get_port_from_instance()` → check instance names in benchmark
   
3. **Inspect grid coordinates**: Do source/target map to unblocked cells?
   - Add logging in `_port_to_state()` before calling `physical_to_grid()`
   
4. **Verify primitives generated**: Do routed paths use expected primitives?
   - Check `primitive_ids` in route result
   
5. **Build fails with Rust errors**: Run `cargo check` first diagnostics

---

## Testing Philosophy

Unit tests per module, with the end-to-end tests marked and separated
(Milestone 6):
- `tests/` holds the unit and integration tests; `tests/e2e/` the tests that
  load a benchmark or a script and run it
- Markers `e2e` and `integration` are registered in `pyproject.toml` and applied
  by `tests/conftest.py`, so `pytest -m "not e2e" tests` is the fast suite
  (under 60 s, enforced by `scripts/test_baseline.sh`)
- Synthetic layouts and sessions are built only in `tests/fixtures/`, exposed as
  conftest fixtures; no test imports a toy benchmark module
- Rust tests live next to their module, with `engine::test_support` and
  `search::test_support` as the shared fixtures

**Pattern**: Create temp `build/` directory for debug artifacts, then validate outputs (SVGs, routing stats).

---

## Version Constraints & Known Issues

- **Python**: ≥3.10 (type hints, match statements in tests)
- **Rust**: 2021 edition (PyO3 0.22 requires recent toolchain)
- **gdsfactory**: Requires `bend_euler_all_angle()` for 45° primitives (fallback to 90° only if unavailable)

Known Issue: `bend_euler_all_angle()` may not exist in older gdsfactory; `primitive_library.py` line 57-65 handles fallback.

---

## Quick Reference: Adding Features

### Add a new routing option
1. Add the field to the typed configuration on both sides: `src/config.rs`
   (`RouterConfig`'s group) and `python/photonic_router/config.py` (its mirror,
   same default), and pass it in `RouterConfig.to_rust` / `src/bindings/types.rs`
2. Read it where the algorithm needs it (never from the environment: `grep -rn
   "env::var\|var_os" src/` must stay empty)
3. Add one `ENV_OVERLAY` entry in `python/photonic_router/env_overlay.py` if it
   should be settable ad hoc, and a `FlowOptions` field plus a flag in
   `flow_options.py` / `cli.py` if it is a flow-level choice
4. Regenerate `docs/CONFIGURATION.md`
   (`.venv/bin/python scripts/generate_config_reference.py`)

### Add a search engine, or a negotiation policy
Both recipes are in `docs/ARCHITECTURE.md` (sections 4 and 5): implement
`NetSearch` in its own module under `src/search/` and name it in
`search_engine_for`; or declare the decision as a trait under
`src/engine/negotiation/`, construct it once before the round loop, and let the
loop ask it.

### Add debug output
1. Write SVG export in Rust (see `export_route_svg()`)
2. Pass to Python, write via `Path.write_text(svg_str)`
3. Add flag to `run_routing_flow(..., debug_option=True)`

### Add new primitive type
1. Define in `src/primitives.rs`
2. Extend `PrimitiveLibraryConfig` in Rust
3. Add corresponding gdsfactory component in `primitive_library.py`
4. Update primitive ID count (currently 56 = 8 angles × 7 types)

---

**Last Updated**: 2026-09-24 (Milestone 7: `docs/ARCHITECTURE.md` and `docs/CONFIGURATION.md`) | **Codebase Version**: 0.1.0 (Rust+Python hybrid)
