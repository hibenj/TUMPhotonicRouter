# Repository State

This file is a compact checkpoint for humans and future agents. Update it at
least every 10 commits and before major handoffs. It does not replace the active
ExecPlan.

## Current Snapshot

- Date: 2026-07-10
- Branch: `baseline/lidar-pure-crossings`
- Baseline before documentation cleanup: `fdc20e4`
- This repository-state checkpoint was introduced by the documentation/workflow
  cleanup commit. Run `git rev-parse --short HEAD` for the current commit.
- Active ExecPlan:
  `.agent/execplans/2026-07-10-crossing-verification-foundation.md`

## Current Goal

Make TUMPhotonicRouter a very fast verified photonic router. The current phase
focuses on router-discovered optical crossings on `benes_4x4`, `benes_8x8`, and
then `multiportmmi_8x8`, with final-geometry verification and PDK/gdsfactory
crossing component realization.

## Worktree State

The worktree is currently dirty. Several modified Rust/Python/test files appear
to come from earlier crossing-routing work. The `.agent/` documentation and
workflow files are also currently untracked or modified.

Current dirty file summary from `git status --short`:

    M AGENTS.md
    M docs/repository_finished_state.md
    M routing_flow.py
    M src/astar.rs
    M src/geometry_realization.rs
    M src/obstacle_map.rs
    M src/py_router.rs
    M tests/test_realized_crossing_verification.py
    M tests/test_rust_backend_import.py
    M tests/test_rust_batch_repair.py
    M translation/route_rust.py
    M translation/route_rust_realization.py
    M translation/route_rust_records.py
    ?? .agent/
    ?? benchmarks/data/
    ?? benchmarks/multiportmmi_8x8.py
    ?? docs/photonic_router_graph_crossing_plm_full.tex
    ?? tests/test_multiportmmi_benchmark.py

Do not revert these files without explicit user instruction.

## Known Documentation Changes In Progress

The new agentic workflow structure includes:

- `.agent/PROJECT_GOAL.md`
- `.agent/WORKFLOW.md`
- `.agent/ORCHESTRATOR.md`
- `.agent/GIT_WORKFLOW.md`
- `.agent/roles/`
- `.agent/execplans/2026-07-10-crossing-verification-foundation.md`

The older multiport LiDAR-style ExecPlan is marked as paused/historical.

## Recommended Next Action

Before implementation, group and commit the documentation/workflow changes
separately from the large existing routing-code changes. Do not use `git add .`
until the commit groups are clear.

Recommended first commit group:

- `AGENTS.md`
- `.agent/PROJECT_GOAL.md`
- `.agent/WORKFLOW.md`
- `.agent/ORCHESTRATOR.md`
- `.agent/GIT_WORKFLOW.md`
- `.agent/REPOSITORY_STATE.md`
- `.agent/roles/`
- `.agent/execplans/2026-07-10-crossing-verification-foundation.md`
- `.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md`
- `docs/repository_finished_state.md`

Suggested commit message:

    docs: add agent workflow and crossing project goal

No code tests are required for that documentation-only commit, but the diff
should be reviewed before staging.

## Next Engineering Step After Git Cleanup

Activate the Planner / Technical Lead role from `.agent/ORCHESTRATOR.md` to
audit:

- crossing metadata creation;
- PDK/gdsfactory crossing component selection;
- crossing footprint to grid keepout/access conversion;
- geometry realization and crossing insertion;
- port snapping;
- final photonic verification;
- A* route-induced insertion-loss cost versus non-physical search penalties.
