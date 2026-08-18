# Repository State

This file is a compact checkpoint for humans and future agents. Update it at
every agent stop, pause, or handoff. It does not replace the active ExecPlan.

Keep this file compact. Do not append a new dated section per stop; overwrite
the sections below in place so this file always reflects only the current
state. Detailed run-by-run history belongs in the active ExecPlan's `Progress`
and `Surprises & Discoveries` sections (per `.agent/PLANS.md`), which are
allowed to grow because they are the plan's own record, not this file's job.
Older history is still recoverable from `git log -p -- .agent/REPOSITORY_STATE.md`
if it is ever needed.

## Current Snapshot

- Date: 2026-08-18
- Branch: `crossings/verification-foundation`
- Current HEAD: `be95ee1` (`routing: split routing_flow.py orchestration into
  focused modules`)
- Working tree is clean (`git status --short` empty).
- Active ExecPlan:
  `.agent/execplans/2026-08-18-restructure-route-nets-rust.md` ("Phase 2")
- Both prior readability plans are complete and committed:
  `.agent/execplans/2026-08-17-restructure-translation-route-rust.md`
  ("Phase 1": 7 commits `60047a3`..`bcce21e`, `translation/route_rust.py`
  11,327 -> 6,865 lines) and
  `.agent/execplans/2026-08-11-refactor-python-routing-flow.md` (`routing_flow.py`
  split, commit `be95ee1`, including the fix for the 4-name monkeypatch
  regression found while validating it). Full test suite is
  `23 failed, 307 passed, 1 skipped`, the confirmed true pre-refactor
  baseline; all 23 failures are pre-existing and unrelated to either plan.
- Phase 2 (the active plan) is planned but not started: a research fork did a
  full structural audit of `route_nets_rust` (the ~6,218-line function with
  ~100 nested closures that Phase 1 deliberately left untouched) and the plan
  document has a concrete 8-milestone sequence. Milestone 1 has not been
  executed yet.

## Current Goal

Make TUMPhotonicRouter a very fast, verified photonic router. The active
phase (per `.agent/PROJECT_GOAL.md`) is router-discovered optical crossings on
`benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`, with final-geometry
verification and PDK/gdsfactory crossing component realization. On 2026-08-17
the user set a prerequisite priority: the codebase is "kinda works but...
all mixed up quite badly" and needs to be readable and structured with known
good practice before more crossing-verification feature work continues. Two
plans toward that objective are done (see Current Snapshot); Phase 2 is the
harder remaining piece, decomposing `route_nets_rust` itself, and is
meaningfully higher-risk than the completed plans since it changes how the
code is written (closures into class methods), not just where it lives.
Once Phase 2 reaches a reasonable stopping point, work should return to the
crossing-verification-foundation objective (see Next Engineering Step).

## Worktree State

Uncommitted change set, per `git status --short` on 2026-08-17:

Clean. Both the `routing_flow.py` split and the Phase 1 `route_rust.py`
extraction are committed (see Current Snapshot). Two known pre-existing,
unrelated test/benchmark failures remain, both already covered by the 23-item
baseline failure list and neither caused by either completed plan:

- `tests/test_routing_flow_stats.py::test_run_routing_flow_collects_route_summary_when_stats_requested`
  fails because `RouteAttemptRecord.as_dict()` now includes `crossing_hotpath_*`
  fields the test's expected literal does not list.
- The default `TOY` CLI benchmark fails at `gc1_to_mmi_in2` with `No route
  found` on this branch. Do not use `TOY` as a smoke test here.

Full history of how the `routing_flow.py` split's regression was found and
fixed (a 4-name monkeypatch-target mismatch, not the 1 originally suspected)
is in `.agent/execplans/2026-08-11-refactor-python-routing-flow.md`'s
`Progress` section; not repeated here since it is closed and this file is
meant to stay compact, per its own header note.

## Recent Session Notes

On 2026-08-17, a Claude Code session (not Codex) added a Claude+Codex
orchestration flow on top of the existing `.agent/` workflow: Claude plays the
Orchestrator/Planner/Explorer/Reviewer/QA roles and delegates only the
Implementation Engineer role to Codex CLI via `.agent/scripts/codex_task.sh`.
See `.agent/CLAUDE_CODEX_FLOW.md` for the mechanics. This is new, unreviewed
by the user for commit, and unrelated to the routing-flow refactor above.

The same session found and fixed staleness in the pre-existing `.agent/` docs
that had accumulated from prior Codex-only sessions: this file had grown to
over 10,000 lines of un-pruned timestamped log entries (now trimmed to this
compact form; full history remains in git), and `.agent/ORCHESTRATOR.md`
named a stale "active ExecPlan" pointer that had fallen behind this file's
own (correct) `Current Snapshot` section. Both are fixed as of this note. A
full audit of the two large ExecPlans
(`.agent/execplans/2026-07-10-crossing-verification-foundation.md`, ~175KB,
and `.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md`, ~59KB)
for the same kind of staleness was explicitly deferred; treat their internal
detail as unverified until that audit happens.

The same session then executed Phase 1 of `2026-08-17-restructure-translation-route-rust.md`
through the newly-added Claude+Codex flow: 7 slices, each written as a task
file with an exact function list and explicit external-compatibility-surface
requirements, handed to Codex via `.agent/scripts/codex_task.sh`, independently
re-verified by Claude (not just trusting Codex's self-report) with `py_compile`,
the full test suite, and a benchmark JSON diff, then committed one slice at a
time. One environment blocker came up on the first Codex call: `bwrap` (Codex's
sandbox) failed with a user-namespace error caused by an Ubuntu 24.04 AppArmor
default; the user fixed it once with a scoped `bwrap`-specific AppArmor profile
(not a blanket relaxation), and it did not recur. Two of the seven task files
Claude wrote incorrectly claimed certain tests "should pass cleanly" when they
were already in the documented baseline; Codex caught both discrepancies and
correctly deferred to the ExecPlan rather than "fixing" pre-existing bugs
outside its scope, which is recorded as a positive signal for the role
boundary but also as feedback that task files need to be cross-checked against
the baseline list before being written, not written from memory.

On 2026-08-18, after Phase 1 and the `routing_flow.py` split were both
committed, the user asked to continue into Phase 2 (`route_nets_rust`
itself). A research fork did a full AST-level structural audit of that
function (shared-state inventory, closure inventory grouped into 7 clusters,
control-flow structure, risk flags, an 8-milestone conversion sequence) and
Claude wrote `.agent/execplans/2026-08-18-restructure-route-nets-rust.md`
from it. Execution has not started; see that plan's `Progress` section.

## Reference Branches

The branch `baseline/lidar-pure-crossings` contains a WIP prototype snapshot
at commit `69ab9fd` ("wip: snapshot experimental lidar-pure crossing
prototype"). Treat it as reference material only, not a merge candidate. If
code is needed from it: audit the relevant diff, confirm the idea fits
`.agent/PROJECT_GOAL.md` and the active ExecPlan, port the smallest useful
piece manually or via a narrow reviewed cherry-pick, and add focused
verification before considering the port complete.

`.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md` is marked
reference-only in its own status note (added 2026-07-10) unless the user
explicitly resumes it.

## Next Engineering Step

1. Execute Milestone 1 of `.agent/execplans/2026-08-18-restructure-route-nets-rust.md`:
   capture that plan's own baseline (full suite plus `benes_4x4`/`benes_8x8`/
   `multiportmmi_8x8`), then scaffold the `_RouteNetsRustSession` class and
   convert the read-only config attributes, as a Codex Implementation
   Engineer task via `.agent/scripts/codex_task.sh`, reviewed and committed
   per `.agent/CLAUDE_CODEX_FLOW.md`. Continue through Milestones 2-4, then
   stop for a user check-in before Milestone 7 (the plan's own built-in
   check-in points; see its Decision Log for why).
2. Triage the two known pre-existing test/benchmark failures noted above
   (stats test literal, `TOY` benchmark) whenever convenient; neither blocks
   Phase 2.
3. Once Phase 2 reaches a reasonable stopping point (or the user decides to
   pause it), decide whether to continue into a Phase 3 for the large Rust
   files (`src/py_router.rs` at 17,399 lines, `src/astar.rs` at 10,388 lines,
   `src/geometry_realization.rs` at 8,125 lines) or resume the
   crossing-verification-foundation objective. Per the last recorded
   stable-benchmark checkpoint in git history (commit `a29dc00`), `benes_8x8`,
   `benes_16x16`, `multiportmmi_8x8`, and `multiportmmi_16x16` are marked
   full-run stable; `multiportmmi_32x32` is not yet, with route 156 / `n_155`
   the last known next slow/hanging route to investigate. Re-verify this is
   still current before acting on it, since it predates this cleanup.
