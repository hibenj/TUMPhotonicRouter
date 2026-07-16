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

### Architecture: 3-Stage Orchestration Pipeline

```
[routing_flow.py] - Main entry point, orchestrates:
1. Load → load_benchmark(name) loads Python schematic from benchmarks/
2. Translate → layout_from_schematic() converts Schematic → Component  
3. Route → route_nets_rust() uses Rust backend to route each net via A*
```

**Key Files:**
- `routing_flow.py` (155 lines) - **Start here**: Full 3-stage pipeline with debug support
- `Agent_implementation_files/ROUTING_FLOW_ARCHITECTURE.md` - Design rationale
- `translation/route_rust.py` (242 lines) - Bridge between Python layout and Rust router

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
- Conversion: `_orientation_to_angle()` in `route_rust.py` normalizes orientation → 0-7

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
python routing_flow.py  # Runs TOY benchmark with debug enabled
```

On Windows, use the project virtualenv executable when present:

```powershell
.\.venv\Scripts\python.exe routing_flow.py
```

Produces:
- `build/static_obstacles/toy_obstacles.svg` - Grid obstacles
- `build/routes/toy_*.svg` - Per-net routing paths
- Opens SVG in browser automatically

### Add New Benchmark

1. Create `benchmarks/MY_DESIGN.py`:
```python
from gdsfactory.schematic import Schematic, Instance, Placement

def build_schematic() -> Schematic:
    schematic = Schematic()
    schematic.add_instance("comp1", Instance(component="grating_coupler_te"), Placement(x=0, y=0))
    schematic.add_instance("comp2", Instance(component="mmi_1x2"), Placement(x=100, y=0))
    # Add nets via schematic.netlist.routes (see benchmarks/TOY.py for pattern)
    return schematic
```

2. Run: `python -c "from routing_flow import run_routing_flow; run_routing_flow('MY_DESIGN', debug_svgs=True, show_klayout=True)"`

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

Tests live in `tests/`:
- `test_rust_backend_import.py` - Verify Rust bindings
- `test_routing_flow_stats.py` - Full pipeline testing
- Pattern: Use `pytest` with temp directories for artifacts

---

## Project-Specific Patterns

### 1. Dynamic Benchmark Loading

Benchmarks are **pure Python functions**, not YAML:
```python
# benchmarks/TOY.py
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
- `route_rust.py` - Production Rust A* router

**Interface contract**: `route_nets_*(unrouted_layout, schematic, ...) -> Component`

Allows swapping routers in `routing_flow.py` without touching other pipeline stages.

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

| File | Lines | Purpose | When to Edit |
|------|-------|---------|--------------|
| `routing_flow.py` | 155 | Pipeline orchestrator | Adding stages, debug options |
| `translation/route_rust.py` | 242 | Python↔Rust bridge | Port conversion, primitive placement |
| `python/photonic_router/primitive_library.py` | 179 | Component library | Adding primitives, bend config |
| `src/astar.rs` | 490+ | A* pathfinding | Algorithm tuning, heuristics |
| `src/obstacle_map.rs` | ? | Grid discretization | Cell packing, clearance metrics |
| `src/primitives.rs` | ? | Primitive definitions | Routing moves, state transitions |
| `Agent_implementation_files/ROUTING_FLOW_ARCHITECTURE.md` | 161 | Design docs | Understanding design intent |
| `benchmarks/TOY.py` | ? | Example benchmark | Template for new benchmarks |

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

**From `route_rust.py`:**
- `GridSpec`: width/height (cells), grid_size_um (0.5), origin
- `PrimitiveLibraryConfig`: grid_size_um (must match GridSpec)
- `AStarConfig`: max_iterations (100k), bend_weight (1.0), target_tolerance_cells (0)

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

Tests are integration-focused:
- `test_rust_backend_import.py`: Verifies bindings exist (smoke test)
- `test_routing_flow_stats.py`: Full end-to-end pipeline
- Tests use `TOY` benchmark as standard fixture

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
1. Extend `AStarConfig` in `src/astar.rs`
2. Pass through `PyPhotonicRouter` binding in `src/py_router.rs`
3. Surface in Python `route_rust.py` line ~191

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

**Last Updated**: May 2026 | **Codebase Version**: 0.1.0 (Rust+Python hybrid)
