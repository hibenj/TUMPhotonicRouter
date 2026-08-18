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

- Date: 2026-08-17
- Branch: `crossings/verification-foundation`
- Current HEAD: `bcce21e` (`routing: extract obstacle config resolution from
  route_rust.py (Slice 7)`)
- Active ExecPlan:
  `.agent/execplans/2026-08-17-restructure-translation-route-rust.md`
- Phase 1 of that plan (all 7 slices, `translation/route_rust.py` module
  extraction) is complete and committed as 7 separate commits (`60047a3`
  through `bcce21e`), each independently verified: full pytest suite exact
  match to the pre-Phase-1 baseline every time, `benes_4x4`/`benes_8x8`
  verification JSON byte-identical every time. `translation/route_rust.py`
  went from 11,327 to 6,865 lines. See that plan's `Outcomes & Retrospective`
  for the full writeup. Its last open Progress item is a user decision on
  whether to start a Phase 2 plan for `route_nets_rust` (~6,200 lines, ~100
  nested closures, deliberately untouched by Phase 1).
- Worktree is still dirty with the uncommitted, not-quite-complete slice of
  the separate `2026-08-11-refactor-python-routing-flow.md` plan (see
  Worktree State below, unchanged since the last note) and the new `.agent/`
  agent-workflow tooling from this session (see Recent Session Notes below),
  neither of which this Phase 1 work touched or depends on.

## Current Goal

Make TUMPhotonicRouter a very fast, verified photonic router. The active
phase (per `.agent/PROJECT_GOAL.md`) is router-discovered optical crossings on
`benes_4x4`, `benes_8x8`, and `multiportmmi_8x8`, with final-geometry
verification and PDK/gdsfactory crossing component realization. On 2026-08-17
the user set a prerequisite priority: the codebase is "kinda works but...
all mixed up quite badly" and needs to be readable and structured with known
good practice before more crossing-verification feature work continues. Two
plans sit between here and that objective, both paused (not abandoned): the
`2026-08-11-refactor-python-routing-flow.md` plan (nearly done) and the new
`2026-08-17-restructure-translation-route-rust.md` plan (not started). Once
those close or reach a reasonable stopping point, work should return to the
crossing-verification-foundation objective (see Next Engineering Step).

## Worktree State

Uncommitted change set, per `git status --short` on 2026-08-17:

- `routing_flow.py` modified; seven new sibling modules added:
  `routing_flow_component_info.py`, `routing_flow_config.py`,
  `routing_flow_electrical.py`, `routing_flow_optical.py`,
  `routing_flow_plm.py`, `routing_flow_reporting.py`,
  `routing_flow_stats.py`, `routing_flow_verification.py`.
- This is the `2026-08-11-refactor-python-routing-flow.md` plan's work: moving
  concerns (stats, verification, PLM, electrical routing, reporting, optical
  routing stage, config/defaults) out of the previously 2622-line
  `run_routing_flow()` into top-level helpers and dedicated modules, while
  keeping the public interface (`run_routing_flow(...)`, `main(argv=None)`)
  unchanged.
- Per that plan's `Progress` checklist, every slice through 2026-08-11 14:15
  local is checked off and validated (`py_compile`, `git diff --check`, and a
  real `heater_s_mod` PLM + electrical-routing run with clean crossing/
  photonic verification JSON after each slice). Only one item remains open:
  optionally extracting CLI parser construction into its own module if
  `routing_flow.py` needs to shrink further.
- Correction, 2026-08-17: the claim that this slice was "essentially
  complete" understated real gaps, found while capturing a baseline for
  `2026-08-17-restructure-translation-route-rust.md`. The full Python test
  suite was never actually run against this slice, only targeted subsets per
  step; running it (`PYTHONPATH=. .venv/bin/pytest -q`) showed 3 names
  missing from `routing_flow.py`'s re-export surface (`_format_debug_route_indices`,
  `_verification_status_metadata`, and a renamed `parse_debug_svg_selector`)
  that blocked test collection outright, now fixed directly since they match
  the plan's own established compatibility pattern. Once fixed, the full
  suite reported `29 failed, 301 passed, 1 skipped`, 6 more than the true
  pre-refactor baseline.
- Fixed, 2026-08-18: the 6-test regression above turned out to be 4 test
  files monkeypatching the wrong module, not 1. Every affected test chains
  several `monkeypatch.setattr(routing_flow, "<name>", fake)` calls, and
  `monkeypatch.setattr` raises immediately on the first missing attribute, so
  fixing `load_benchmark_metadata` alone just exposed the next mistargeted
  name in the same test rather than making it pass. Found the full set with a
  script that regex-scans every `monkeypatch.setattr(routing_flow, "...")`
  call (including multi-line ones) across `tests/*.py` and cross-checks each
  name against `hasattr(routing_flow, name)`. All 4 mistargeted names, and
  the sibling module that now actually owns each one: `load_benchmark_metadata`
  and `route_match_and_realize` -> `routing_flow_optical`;
  `verify_photonic_routing` -> `routing_flow_verification`;
  `route_electrical_heaters` -> `routing_flow_electrical`. Fixed by
  retargeting all 8 call sites to the module that actually owns the call, per
  explicit user direction that this is what actually matches the
  restructuring's own goal (each module tested against its own real
  dependency), not by routing calls back through `routing_flow.py` for
  compatibility. `PYTHONPATH=. .venv/bin/pytest -q` now reports
  `23 failed, 307 passed, 1 skipped`, an exact match to the true pre-refactor
  baseline. One test does not start passing despite being on the original
  regression list, correctly: `test_path_length_graph.py::test_main_flow_flag_enables_path_length_matching`
  was already failing on clean HEAD for a real, unrelated, pre-existing bug
  (`TypeError` in `translation/route_rust_realization.py:324`) that the
  mistargeting had been masking; it now fails with that real error instead.
  Full detail in `.agent/execplans/2026-08-11-refactor-python-routing-flow.md`'s
  `Progress` section. This slice can now honestly be called complete pending
  only the optional CLI-parser item above. Not yet committed; see below.
- Known pre-existing, unrelated failure (one of the 23 above):
  `tests/test_routing_flow_stats.py::test_run_routing_flow_collects_route_summary_when_stats_requested`
  fails because `RouteAttemptRecord.as_dict()` now includes `crossing_hotpath_*`
  fields the test's expected literal does not list.
- Known pre-existing, unrelated failure: the default `TOY` CLI benchmark
  fails at `gc1_to_mmi_in2` with `No route found` on this branch. Do not use
  `TOY` as a smoke test here.
- None of this dirty state has been committed. Follow `.agent/GIT_WORKFLOW.md`
  commit-boundary guidance before committing: the refactor slice, its
  regression fix (once decided), and the new `.agent/` tooling below are
  separate concerns and should not land in one commit.

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

1. Decide with the user whether to start a Phase 2 ExecPlan for
   `route_nets_rust` (the ~6,200-line, ~100-nested-closure function
   deliberately left untouched by Phase 1), continue into Phase 3 (the large
   Rust files: `src/py_router.rs` at 17,399 lines, `src/astar.rs` at 10,388
   lines, `src/geometry_realization.rs` at 8,125 lines), or pause the
   readability work here. This is the open item in
   `.agent/execplans/2026-08-17-restructure-translation-route-rust.md`.
2. Close out or explicitly drop the one remaining optional item in
   `.agent/execplans/2026-08-11-refactor-python-routing-flow.md` (CLI parser
   extraction), and fix or explicitly accept its documented 6-test
   `load_benchmark_metadata` regression before committing that slice.
3. Triage the two known pre-existing test/benchmark failures noted above
   (stats test literal, `TOY` benchmark) separately from both refactors,
   since neither is caused by them.
4. Once the readability work reaches a stopping point the user is satisfied
   with, resume the crossing-verification-foundation objective. Per the last
   recorded stable-benchmark checkpoint in git history (commit `a29dc00`),
   `benes_8x8`, `benes_16x16`, `multiportmmi_8x8`, and `multiportmmi_16x16`
   are marked full-run stable; `multiportmmi_32x32` is not yet, with route
   156 / `n_155` the last known next slow/hanging route to investigate.
   Re-verify this is still current before acting on it, since it predates
   this cleanup.
