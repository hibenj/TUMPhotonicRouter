# Orchestrator Agent Guide

This file tells a lead/orchestrator agent how to coordinate work in this
repository. It is not a replacement for `AGENTS.md`, `.agent/PROJECT_GOAL.md`,
`.agent/WORKFLOW.md`, or the active ExecPlan. It is the operating guide for the
agent that decides which role works next.

## Required Startup

At the start of every substantial orchestration session, the orchestrator must
read:

1. `AGENTS.md`
2. `.agent/PROJECT_GOAL.md`
3. `.agent/WORKFLOW.md`
4. `.agent/PLANS.md`
5. `.agent/GIT_WORKFLOW.md`
6. `.agent/REPOSITORY_STATE.md`
7. The active ExecPlan under `.agent/execplans/`
8. `.agent/PATH_INVESTIGATION_HARNESS.md` -- mandatory procedure for every "net X does not route" investigation (state pinning, attempt identification by `seq`, pre-failure GDS, geometry-first expected path, seq-filtered diagnostics, post-search validation check, 3-minute per-net watchdog); written 2026-09-03 from that day's mistakes.

**Latest (2026-09-07 18:30): `.agent/execplans/2026-09-07-distributed-crossing-structure.md`** -- generality sweep on a synthetic permutation benchmark (70 configs: all accepted ones route 0/0; loud rejections for narrow bands and for random permutations of widely spaced single-port lanes), corners of column-grid lanes now pre-wired, all seven benchmarks 0 failures / 0 repairs; uncommitted. Previous (15:40): same plan -- contribution 2 covers all seven benchmarks (Benes: X tiles on level columns; multiportmmi: axis-aligned column grid on the router's fan-out anchors), one flag, all verification-clean, uncommitted. Earlier that day: -- contribution 2's structure is now crossing TILES only (owner: pre-place the crossings, the normal router connects), default `fan_mode="tiles"`, positions per the stretched-array rule; all four Benes sizes clean with 0 failures / 0 repairs, benes_32x32 3 % over lidar-pure length, A* 3.9 s (verifier 94 s = follow-up); `stretched` and `router` modes stay selectable; uncommitted. Previous COMPLETE (2026-09-07 10:30): `.agent/execplans/2026-09-07-preplaced-crossing-grids-all-benes.md` -- contribution 2 (`--preplaced-crossing-grids true`, one flag) routes all four Benes sizes verification-clean incl. benes_32x32 (30 s vs ~15 min lidar-pure); defaults: lane pitch 14 um, span net order for grid mode, stable-block crossing flags dropped; baseline verdicts unchanged; uncommitted at the time of writing. Owner direction 2026-09-07: all contributions must work first (contribution 1 = S1 as is), then return to contribution 1 improvements. Previous COMPLETE (2026-09-04 17:15): `.agent/execplans/2026-09-04-crossing-guided-search.md` -- contribution 1 landed (`a94750b`): `--crossing-mode lidar-guided`, opt-in, 34-45 % fewer A* expansions on all six crossing benchmarks with identical verdicts; S2 (per-pair budget, off by default) and S3 (`--net-order`, opt-in; `plan-crossings-desc` wins the ladder but hangs benes32's middle stage) landed the same evening; open owner question: hybrid order / net-282 investigation. Standing rule (owner): exactly one of baseline lidar-pure / contribution 1 (guided A*) / contribution 2 (preplaced crossing structures, `--preplaced-crossing-grids`) runs at a time; nothing of the contributions leaks into the lidar-pure defaults. Latest COMPLETE (2026-09-04 13:10): `.agent/execplans/2026-09-04-chip-boundary-keepout.md` -- no routing outside the die (static bands over the grid padding + `route_outside_chip` verifier gate), all benchmarks green. Previous COMPLETE (2026-09-04 11:12): `.agent/execplans/2026-09-04-mm32-first-layer-no-crossings.md` -- braid repair at commit (`a1ffa32`); first layer crossing-free, all benchmarks green. **PARKED (owner, 2026-09-04 10:30): `.agent/execplans/2026-09-04-predicate-1-one-number.md`** -- predicate 1 restructured into a difference matrix (tests pinned to baseline verdicts, `5e99978`) with Step 2 = one relaxation per slice; the rule code on HEAD equals `baseline-2026-09-03` (tag). First attempt on branch `pred1-rework-wip`. Resume only on the owner's word. No active ExecPlan until the owner names the next work item. Previous: `.agent/execplans/2026-09-03-performance-knobs-p1-p5.md` -- COMPLETE: four rule-free performance changes, mm32 490 -> 298 s, benes32 945 -> 815 s, all verdicts identical; open owner questions listed in the benes plan / repository state (P4 structural search cost, predicate-1 simplification). No active plan until the owner decides. Previous, COMPLETE: `.agent/execplans/2026-09-03-benes-32x32-lidar-pure-baseline.md` -- (benes_32x32 320/320 verification-clean in 945 s; both 32x32 baselines reached; retrospective and the seven-benchmark performance pass recorded). Its performance findings P1 (obstacle-map clone in endpoint correction) and P5 (non-routing phases) are the owner's next work items; a fresh ExecPlan is expected for them. `.agent/PATH_INVESTIGATION_HARNESS.md` is mandatory for any route investigation. Previous plan of the same day: `.agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md` -- COMPLETE through Milestone 4 (consolidated 2026-09-03): `multiportmmi_32x32` routes completely and verification-clean (447/447, 0 failures, 0 repairs, 469 s), ladder green, decisive fix = crossing-window disjointness enforced inside the search (predicate 2). One OPTIONAL follow-up is listed in its Progress (predicate-1 simplification: one number + explicit fillet trims, deletions of `bend_runout`/`required_margin`/dead knob); it is not scheduled -- the repository owner decides what comes next. The plan's Rule audit and Retrospective are the reference for how crossing legality is enforced and for the process lessons (per-search `seq` before analysing a failure line; every post-search reject must be traced and mirrored as a search-time rule). Its parent, `.agent/execplans/2026-09-01-forced-90-degree-route-degradation.md`, holds the 2026-09-01/02 history. The repository owner is directing work turn by turn, not delegating to an autonomous orchestration loop; no subagents for complex tasks by owner direction. Several older plans exist; do not resume any of them automatically without the repository owner's direction.

`.agent/execplans/2026-08-25-negotiated-repair-engine.md` (written
2026-08-25) is **complete, all 8 milestones**, with a materially honest,
narrower-than-originally-hoped outcome stated plainly in its own Outcomes
& Retrospective. It grew directly out of the kernel-unification plan
below finding that none of the old chain's 17 methods are dead code,
which the repository owner correctly pointed out is a correctness bar,
not a design bar -- "just becaus[e] all these strategies are somewhat
called doesnt mean the[y] are really necessary or that the behaviour is
kinda 'wild'." Real, general improvements landed directly in the
still-default dispatch chain (topological net ordering, systematic
per-commit history cost); a complete second, opt-in, faster negotiated-
congestion repair engine was built but not shipped as a default, since
`benes_16x16` needs crossing-legality-aware repair strategies it does
not implement. The single highest-value finding came *after* that
engine work, prompted by the repository owner questioning the
assistant's own proposed next step: `--proactive-congestion-weight`/
`--proactive-congestion-radius-cells`, a general mechanism that was
already fully built and simply never turned on anywhere, resolves
`benes_8x8`/`benes_16x16`/`multiportmmi_8x8` outright with zero source
changes (now part of those benchmarks' own `STABLE_ROUTING_FLAGS`, not
a global default -- it regressed 3 unrelated cases when tried as one).
`multiportmmi_16x16` remains a known, documented gap under every
configuration tried -- see `.agent/REPOSITORY_STATE.md`'s Current
Findings and Next Engineering Step for the exact reproduction command
and the most promising untried direction (crossing-aware rerouting
inside the negotiation loop, identified and deferred in this plan's own
Milestone 6).

`.agent/execplans/2026-08-25-unify-astar-kernel-and-clean-repair-baseline.md`
(written 2026-08-25) is complete through Milestone 5 of 6: the two
duplicate ~600-900 line A* search kernels in `src/astar.rs` are unified
into one, with crossing-legality checking as a collision-triggered,
pluggable hook; every production call site migrated; the old kernels
deleted with zero remaining references, validated byte-identical
against the full benchmark ladder (a real bug -- `straight_run_cells`
not tracked for Tier-1 states -- was found and fixed by that same
ladder, not by a unit test). Milestone 5 investigated the 19-strategy
(now 17) pre-repair dispatch chain and found most of it load-bearing,
not duplication -- only 2 undocumented, off-by-default, no-rationale
experiments (`try_guided_collision_crossing`, `try_preemptive_crossing_ripup`)
were actually deletable, and were deleted after the repository owner
reviewed the git-history evidence directly. Only Milestone 6 (final
validation ladder, `REPOSITORY_STATE.md` update, retrospective) remains
open, superseded in immediate priority by the negotiated-repair-engine
plan above, which grew out of this plan's own Milestone 5 finding.

`.agent/execplans/2026-08-24-endpoint-correction-cascade-soundness.md`
(written 2026-08-24) is paused mid-Milestone-4. Milestone 3 (removing the
unchecked endpoint-correction fallback cascade) is implemented and
committed; validating it found a larger real-world consequence than
Milestone 2's own measurement predicted (5 nets newly fail in
`multiportmmi_8x8` stable-baseline, not 1), which the repository owner
has accepted for now. Paused at their direct request to read through the
routing pipeline manually, starting at `routing_flow.py`, before deciding
whether/how to continue this plan's Milestone 4 -- that manual walkthrough
is what produced the new kernel-unification plan above.

`.agent/execplans/2026-08-25-python-rust-linting-and-coding-standards.md`
(written 2026-08-25) is paused after Milestone 2 at the repository
owner's direct instruction, deliberately not completed further:
Milestones 1-2 (tool/config decisions, the coding-standards document,
one repository-wide formatting-only pass) are done and committed;
Milestones 3-5 (safe auto-fixes, hand-fixing or justified-suppressing
the remaining ~800 lint findings, blocking CI) are deferred, not
abandoned, in favor of functional/edge-case test coverage -- see that
plan's own Decision Log for the owner's exact words and `.agent/CODING_STANDARDS.md`
for what Milestone 1 produced.

`.agent/execplans/2026-08-24-crossing-cost-function-soundness.md` is
complete (all 3 milestones, 2026-08-24). Fixed two confirmed logical-
soundness gaps found while investigating a repository-owner-reported visual
anomaly (90-degree-only bends on `n_7`/`n_8` in `multiportmmi_8x8` stable-
baseline): (1) `bend_weight`/`heuristic_weight` no longer silently vary by
`crossing_mode`; (2) `require_terminal_straights` is now honored by the
crossing-aware kernel too, not just the plain one. Along the way, validating
those fixes surfaced and fixed a real, pre-existing illegal-geometry-
acceptance bug in the endpoint-correction cascade (commit `2755a22`), and
caught a methodology mistake (both this plan's own timing investigation and
an earlier `stabilize-16x16-benchmarks` claim had used the wrong, non-
representative bare-defaults config for `benes_8x8`/`benes_16x16` -- caught
by the repository owner, corrected in both plans and `REPOSITORY_STATE.md`).
Two things remain open, deliberately, per repository owner instruction: see
that plan's own Outcomes & Retrospective, and `REPOSITORY_STATE.md`'s Next
Engineering Step for the endpoint-correction-cascade and `n_49`/`n_50`
candidates this work surfaced.

`.agent/execplans/2026-08-24-stabilize-16x16-benchmarks.md` remains paused
mid-Milestone-1 (root-causing `multiportmmi_16x16`'s repair-exhaustion
failure) from earlier the same day -- not resumed. Note before resuming: the
failure signature moved from `n_50` to `n_49` while validating the plan
above; see that plan's own Milestone 0 correction note for the exact new
error text.

`.agent/execplans/2026-08-24-wire-single-net-search-trait-into-production.md`
is complete (all 3 milestones, 2026-08-24): every production single-net
search call site in `src/py_router.rs` -- 12 sites across 9 methods,
confirmed by direct census, not the "8" the modular-routing-strategies
plan had estimated while deferring this exact work -- now goes through
the `SingleNetSearch` trait via `AStarSingleNetSearch` instead of calling
the underlying free functions directly. Dispatched to Codex as a single
task with the exact substitution spelled out per site (Milestone 1's own
direct read of all 12 sites found zero argument-shape discrepancies);
zero review findings on the returned diff, the cleanest Codex dispatch of
either plan this session needed. Does not add a second search algorithm
-- none exists yet, none was invented just to prove the point.

`.agent/execplans/2026-08-24-modular-routing-strategies.md` is complete
(all 8 milestones, 2026-08-24): `_RouteNetsRustSession.run()`
(`translation/route_rust.py`) went from a ~1070-line single method to a
37-line ordered sequence of 8 named, documented phase calls; the
windowed-bounds-expansion retry loop duplicated across all four
`src/astar.rs` single-net search wrapper functions is now one shared
generic helper, implemented by Codex and reviewed line-by-line before
acceptance; all 18 `try_*` repair methods in
`route_many_with_repair_and_commit` are individually documented, and the
one pair confirmed to share real structure is now one method instead of
two, per the repository owner's own choice among concrete options Claude
presented (not a full cross-18-method abstraction, which the evidence did
not support). Read that plan's own Outcomes & Retrospective for the full
detail, including a recurring lesson worth reading before starting the
next plan: nearly every milestone's design stage overturned or refined
something an earlier stage of that *same plan* had assumed.

Prior pointer, kept for context:
`.agent/execplans/2026-08-19-fix-open-repair-and-dense-port-findings.md`
(start at Milestone 0). The repository owner's direction (2026-08-19):
fix the open, previously-parked benchmark findings first, then move on
to the Future Architecture Initiative. Three findings in scope: the
`multiportmmi_8x8` bare-defaults `n_67`/`n_70`/`n_71` repair-congestion
cluster, `multiportmmi_16x16` stable-baseline's `n_50` finding (same
outer symptom, not yet confirmed to share a root cause), and the
dense-port lateral-width allocation problem (a design question, not a
mechanical fix). This plan also follows the repository owner's
2026-08-19 direction to use the Claude+Codex flow
(`.agent/CLAUDE_CODEX_FLOW.md`) for implementation slices going forward
-- diagnosis stays with Claude, but once a fix is bounded and
well-specified, dispatch it to Codex via `.agent/scripts/codex_task.sh`
rather than implementing directly.

`.agent/execplans/2026-08-19-restructure-crossing-partner-discovery.md`
is complete (all 4 milestones, 2026-08-19): the crossing-partner-discovery
tangle (seven overlapping "candidate partner" functions, three
independent decision sites) is now two clean base-case functions, one
shared bbox-filter helper, and one shared, explicitly-parameterized
decision function for the one real, confirmed-intentional divergence
between fresh and repair routing -- with `route_many_with_repair_and_commit`'s
own five usage contexts deliberately left alone, since Milestone 0 found
no evidence they needed restructuring. Read that plan's own Outcomes &
Retrospective for the full detail.

Three prior plans completed earlier the same day:
`.agent/execplans/2026-08-19-restructure-port-endpoint-correction.md`
(all 6 milestones): endpoint correction is now one classification
function feeding two named orchestration entry points, with every
fallback visible in structured verification JSON instead of silently
discoverable only via a geometric audit; along the way it also fixed a
real, confirmed physical waveguide-overlap bug on `multiportmmi_16x16`.
Its follow-up, `.agent/execplans/2026-08-19-collision-avoiding-endpoint-correction.md`
(all 4 milestones): the two nets left honestly-failing by that fix
route cleanly too now, via a collision-check added to an existing
multi-candidate search. A third, `.agent/execplans/2026-08-19-fix-collision-crossing-zero-event-acceptance.md`:
fixed the one pre-existing failing Rust unit test
(`try_route_with_collision_crossings_using_primitives` was accepting a
route with zero crossing events as a successful collision-crossing
result) and, at the repository owner's direction, also fixed a second,
separate, pre-existing native-repair bookkeeping bug
(`restore_saved_source_layer_routes` could silently lose a net's route
on a partial restore failure). **Important**: that third plan's own
benchmark validation was incomplete -- it did not recheck
`multiportmmi_16x16`, which is now confirmed (via the redirect
discussion, see `.agent/REPOSITORY_STATE.md`'s "Correction" bullet) to
have been broken by the zero-event-acceptance fix. Both this and the
`multiportmmi_8x8` bare-defaults finding from that same plan are tracked
as separate candidates in `.agent/REPOSITORY_STATE.md`'s "Next
Engineering Step", not resolved.

All four plans grew out of five prior, now-complete
plans (`.agent/execplans/2026-08-18-*.md`) that together unified port
access/keepout sizing, fixed two real dense-port and endpoint-correction
bugs, and recalibrated the Rust crossing test suite. Read
`.agent/REPOSITORY_STATE.md`'s "Current Snapshot" and "Next Engineering
Step" sections for the current summary and the candidate list for what to
do next -- the repository owner has not yet chosen the next objective, so
do not assume one; that file is kept current at every stop and is the
source of truth for what is actually active, not this paragraph.

Once this plan completes (or a different direction is chosen), update this
pointer to name whatever is active next, the way it has for every plan so
far.

If the user explicitly resumes another ExecPlan, use that plan instead and note
the switch in the resumed plan's `Decision Log`.

## Orchestrator Responsibilities

The orchestrator owns coordination, not every implementation detail. It should:

- Keep the repository goal and active ExecPlan aligned.
- Decide which software-team role should act next: planner, implementation
  engineer, QA/harness engineer, or reviewer.
- Give subagents concrete, bounded tasks with explicit file scopes.
- Prevent overlapping edits by assigning disjoint write scopes.
- Keep the active ExecPlan updated after important findings, decisions, tests,
  and handoffs.
- Keep `.agent/REPOSITORY_STATE.md` updated before every agent stop, pause, or
  handoff.
- Integrate or reject subagent outputs after review.
- Protect the current `lidar-pure` / router-discovered crossing path from
  topology-precomputed crossing hints.
- Require structured verification evidence before declaring routing progress.
- Treat `baseline/lidar-pure-crossings` and its WIP prototype snapshot as
  reference material only. Do not merge that branch wholesale into the clean
  `crossings/verification-foundation` implementation path.

The orchestrator must not treat subagent output as automatically correct. It
should review diffs, check assumptions, and run or request validation.

## When To Spawn Subagents

Use real subagents when they are available and the work can be split without
blocking the immediate next local step. Good delegation examples:

- Ask an explorer to audit where port snapping can move route centerlines.
- Ask an explorer to audit how `total_cost`, bend penalties, crossing loss,
  history, and congestion are currently computed.
- Ask a harness worker to add a focused verification fixture while the main
  agent audits benchmark commands.
- Ask a reviewer to inspect a completed patch against the active ExecPlan.

Do not spawn subagents merely to create the appearance of parallel work. Keep
work local when the next step is tightly coupled, urgent, or hard to specify.

If real subagents are not available, the orchestrator should run the same roles
sequentially in one session using the role briefs in `.agent/roles/`.

**Superseded 2026-08-28 by repository-owner direction: Codex is no longer
used. The Claude agent implements directly and may spawn Claude subagents
for bounded coding and verification lanes; `.agent/CLAUDE_CODEX_FLOW.md` is
kept for history only.**

**Refined 2026-08-31 by repository-owner direction (token economy): the
lead/orchestrator session runs on the most capable model (Fable), whose
usage limit is scarce; it must conserve itself for analysis and hand
everything else to cheaper-model subagents.** Division of labor:

- The lead keeps: root-cause analysis and interpretation of evidence,
  design decisions and owner-option framing, task specification, diff
  review of returned work, the final validation verdict, ExecPlan and
  `.agent/REPOSITORY_STATE.md` upkeep, and anything decision-gated.
- Delegate to fresh `general-purpose` subagents with an explicit cheaper
  `model` override (`sonnet` for coding/investigation lanes, `haiku` only
  for trivial mechanical chores): bounded implementation slices with a
  full spec, evidence gathering (run a reproduction, extract the named
  diagnostics/log fields, return a compact report), and validation-ladder
  runs that end in a small results table. Their tool output stays out of
  the lead's context, which is the point.
- Do NOT use `subagent_type: "fork"` to save tokens -- a fork inherits the
  lead's context but runs on the lead's model, so it spends the scarce
  budget instead of conserving it.
- The Codex-era delegation lessons carry over unchanged (they are about
  bounded specification, not about the tool): delegate only
  fully-specified tasks; cheaper models are unreliable at discovering
  "there is already a mechanism for this" on architectural work; if a
  lane does not converge within one or two attempts, pull the work back
  to the lead instead of iterating; independently verify a subagent's
  claims (read the diff, re-run or spot-check the decisive command)
  before committing; and never let a subagent make or fabricate a
  repository-owner decision.
- Subagent prompts must be self-contained the way ExecPlans are
  self-contained: name the files, the exact commands, the expected
  observations, and the report format. Pointing the subagent at the
  active ExecPlan plus a delta brief is the normal shape.

Two standing lane definitions exist as project agents under
`.claude/agents/` and should be preferred over ad-hoc `general-purpose`
spawns, because they carry the guardrails below in their own system
prompts instead of relying on every brief to restate them:

- `evidence-lane` (sonnet, read-only tools): reproduction runs, log/
  diagnostic extraction, verification-ladder measurements. Facts only,
  compact report, no decisions.
- `impl-lane` (sonnet): fully-specified implementation slices; stops and
  reports on any brief-vs-reality discrepancy instead of improvising;
  never commits; returns a verifier packet.

### Subagent harness learnings (append here as they occur)

- (2026-08-31) A lane given a ~6-minute benchmark run started it and then
  ended its turn "to wait for the monitor", stranding the deliverable
  until the lead nudged it. Long commands must run in the foreground with
  an explicit `timeout`, and the lane must not end its turn before the
  report exists. Now baked into both lane definitions; when writing an
  ad-hoc brief, restate it.
- (2026-08-31, same day, second and third occurrence) The stall repeated
  even with the contract text read at task start -- the tempting failure
  shape is `run_in_background` plus a "watchdog" poller, which also keeps
  generating stale wake-up notifications long after the lane delivered.
  Both lane definitions now ban `run_in_background` outright. Also
  learned: a lane's *verifier-packet on failure* can be the most valuable
  outcome -- the impl lane's instrumented "brief-vs-reality" report (the
  real suffix geometry vs the brief's assumed shape) redirected the whole
  fix; and layered validation gates mean a lane's "implemented exactly as
  specified, still red" is normal, not lane failure. Budget for the lead
  to iterate the last mile on gate-by-gate findings (three gates here:
  candidate generation, static old-core exemption, terminal tangent).
- (2026-08-31) Lane briefs must state how benchmark stable defaults
  interact with env overrides (`STABLE_ROUTING_ENV` is applied via
  `os.environ.setdefault`, so an explicitly exported variable wins) —
  otherwise a lane can silently measure the wrong configuration.
- (2026-08-31, root cause of all three same-day stalls) The Bash TOOL's
  own `timeout` parameter (default ~2 minutes, max 600000 ms) is separate
  from a shell-level `timeout NNNN` prefix; when the tool timeout elapses
  on a long benchmark command, the harness auto-backgrounds the call and
  the lane then "waits" -- stranding the deliverable. Lanes must set the
  tool-level timeout explicitly on every long command and split >10-min
  work into start-plus-blocking-wait-loop calls. Now in both lane
  definitions.
- (Codex era, still true for cheaper-model lanes) Fully-specified tasks
  converge; architectural discovery does not — lanes invent bolt-on
  mechanisms instead of finding the existing one. Root-causing and
  design stay in the lead. The previous rule was: when the orchestrator is a
Claude agent (Claude Code) working alongside Codex CLI, use
`.agent/CLAUDE_CODEX_FLOW.md` instead of spawning further Claude subagents
for the Implementation Engineer role: Claude keeps the Orchestrator,
Planner, Explorer, Reviewer, and QA/Harness roles, and delegates only the
scoped coding step to Codex via `.agent/scripts/codex_task.sh`. The Subagent
Prompt Contract below still applies; the script attaches it automatically.

### Mandatory Routing Verification Gate

After every nontrivial routing implementation, the orchestrator must hand off
to QA / Harness Engineer before notifying the user that the slice is done or
materially improved. Nontrivial means any change to routing behavior, crossing
legality, realization, verification semantics, Python/Rust boundaries, A*
repair behavior, benchmark-specific routing logic, or multi-file routing state.

The implementation lane must produce a verifier packet: changed files, exact
commands run, artifact paths, benchmark and route/net identifiers, blocker set
when available, and the current failure signature. The QA / Harness Engineer
must inspect structured reports and relevant generated artifacts directly and
return one of `PASS`, `FAIL`, `BLOCKED`, or `INCONCLUSIVE`. If real subagents
are unavailable, the orchestrator must run an explicit sequential Harness role
pass and label it in the active ExecPlan.

The orchestrator may continue implementation only after recording the verdict
in the active ExecPlan or `.agent/REPOSITORY_STATE.md`. A reviewer should treat
missing harness sign-off as blocking for nontrivial routing work.

### Serial-To-Parallel Escalation

Do not let a repeated benchmark/debug loop grow into one long single-window
prompt. Fan out into bounded subagents or explicit sequential role passes when
any of these triggers occur:

- The same benchmark or focused route stop fails twice after materially
  different fixes or hypotheses.
- A failure signature moves rather than disappears, for example an illegal
  crossing shifts location while preserving the same underlying cause.
- The next step can be split into verifier semantics, Rust/Python repair logic,
  benchmark evidence, and review without overlapping edits.
- A change touches crossing legality, final verification, route realization,
  Python/Rust commit semantics, or A* repair behavior and the fix is no longer
  a tiny local patch.
- The main agent is spending more time running evidence loops than integrating
  conclusions.
- The user reports invalid geometry in a screenshot or GDS after the agent
  claimed the artifact was clean.
- A GDS/debug artifact was written from a partial route stop and could be
  mistaken for a full benchmark result.

When this escalation triggers, the orchestrator should create a small parallel
bundle instead of continuing serially:

- **QA / Harness Engineer** owns verifier semantics, focused fixtures,
  structured report fields, and failure classification.
- **Explorer / Codebase Audit** owns read-only tracing of the relevant code
  paths, including baseline-branch comparison when useful.
- **Implementation Engineer** owns the smallest assigned source edit.
- **Benchmark / Evidence lane** may be handled by the harness role: run the
  agreed route stop or benchmark repeatedly and report only pass/fail, timing,
  route index, net name, blocker set, and changed error signature.
- **Reviewer** inspects risky diffs while benchmark runs are in flight, with
  special attention to whether the implementation weakened legality checks or
  slowed the fast path.

The orchestrator should integrate these outputs and decide the next loop. Do
not notify the user merely because one lane finishes if another lane can
continue, but do record durable findings in the active ExecPlan.

### Verifier Veto And Artifact Freshness

For routing geometry work, the verifier/harness lane has veto power over a
claimed fix. If QA or the user finds visible invalid geometry, the orchestrator
must treat the previous validation packet as insufficient even when focused
tests passed. The next loop must first answer two questions:

1. Which deterministic check should have caught this geometry?
2. Is the inspected artifact current, and is it full or a debug-stop partial?

Before asking the user to inspect a GDS or image artifact, report the command
that produced it, whether `debug-stop-after-route` was active, the routed-record
count from verification JSON when available, the file timestamp, and the
remaining known blockers. Do not call a GDS "clean" without stating whether it
is a full benchmark output or a partial debug artifact.

If a visual/GDS issue repeats after a claimed fix, stop heuristic tuning and
spawn or run these lanes before further behavior edits:

- **QA / Harness Engineer**: reproduce or classify the issue from JSON/GDS/SVG
  artifacts and propose the missing deterministic check.
- **Explorer / Codebase Audit**: trace the failing path without editing.
- **Reviewer**: inspect whether the current diff weakened legality, confused
  partial and full artifacts, or relied on screenshots instead of structured
  evidence.

The orchestrator may continue only after these outputs are integrated into the
ExecPlan or repository state.

For rip-up/reroute failures, the orchestrator must challenge every victim in
the repair set. A net may be a victim only when a trace, report, obstacle owner
lookup, illegal-crossing record, reservation conflict, or focused fixture gives
geometry-backed evidence. Native route order, netlist adjacency, or visual SVG
sequence must never be treated as blocker evidence.

### Command Runtime Guardrails

The orchestrator must not leave validation, benchmark, build, or diagnostic
commands running without an explicit time budget. Before starting any command
that might run longer than a few minutes, record the intended cap in the
working update or ExecPlan and poll it regularly. For routing benchmarks and
debug-stop runs, the default local cap is 5 minutes unless the user explicitly
approves a longer run; for full benchmark validation, state the longer cap
before starting.

If a command exceeds its cap or stops producing useful evidence, terminate only
the matching repository process, record whether any partial artifacts were
created, and report the stop status to the user. Do not start multiple
long-running benchmark variants in parallel unless each lane has its own cap
and process-cleanup plan. A route stop that hangs is a failure signature, not
background progress.

If a turn is interrupted, resumed, or compacted while commands may still be
running, the first action on resume is to inspect for stale repository
processes (`routing_flow.py`, `pytest`, `cargo`, `maturin`, `rustc`) and stop
only stale validation/build processes before continuing.

Use the full multi-agent pipeline at milestone boundaries, for behavior-changing
router work, and for changes that touch crossing legality, route realization,
verification semantics, Python/Rust boundaries, or performance-sensitive A*
logic. Do not require the full pipeline for tiny documentation, harness wiring,
or localized cleanup when focused validation is enough.

## Model And Reasoning Policy

Use the same model for the orchestrator and spawned subagents by default. If the
subagent tooling inherits the parent model when no override is supplied, omit
the model field. Only set a different model when the user explicitly asks for
one or when a narrow task-specific reason is documented in the active ExecPlan.

Vary reasoning effort by role and risk:

- Orchestrator: `high` for substantial coordination and milestone decisions.
- Planner / Technical Lead: `high`, especially before cross Rust/Python routing
  or verification changes.
- Explorer / codebase audit: `medium` for bounded file/path questions; `high`
  for architecture questions that cross routing, realization, verification, or
  the Python/Rust boundary.
- QA / Harness Engineer: `high` when designing verification strategy or failure
  classification; `medium` when running known validation commands and recording
  evidence.
- Implementation Engineer: `medium` for scoped edits; `high` when touching A*,
  crossing legality, route realization, PyO3 bindings, or shared verification
  behavior.
- Reviewer: `high` by default.
- Routine status, documentation cleanup, and mechanical bookkeeping: `low` or
  `medium`.
- `xhigh` or stronger settings: reserve for long-horizon, ambiguous, or
  repeatedly failing work where prior `medium` or `high` attempts did not
  produce actionable evidence.

If the current runtime cannot configure model or reasoning effort for subagents,
state that limitation before delegating. Do not silently downgrade critical
planning, review, or verification work for cost or latency; first establish
correctness evidence, then optimize.

## Standard Orchestration Loop

Use this loop for the current crossing work:

1. Read the active ExecPlan and identify the next unchecked progress item.
2. Decide whether the next step is planning, implementation, review, or harness
   work.
3. If delegating, prepare a role brief that includes:
   - files the subagent must read;
   - files it may edit, if any;
   - files it must not edit;
   - exact output expected;
   - validation commands, if applicable.
4. Start independent lanes together when possible: for example, a harness lane
   can run benchmark evidence while an explorer audits repair logic and a
   reviewer inspects the latest verifier diff.
5. Continue only non-overlapping local work while subagents run.
6. Review returned results before integrating them.
7. Update the active ExecPlan with progress, discoveries, decisions, and
   validation evidence.
8. When the implementation changes routing behavior or final geometry, run the
   verifier/harness pass and reviewer pass before claiming success. A benchmark
   command finishing is not enough; the evidence packet must match the claimed
   artifact.
9. Repeat implementation, QA, and review passes until the current objective
   converges or a documented blocker prevents more local progress.
10. End with a concise status: changed files, tests run, remaining risk, and next
   role to activate.

## Convergence And Failure Policy

The orchestrator should keep working inside the agreed objective until the
objective is validated, reviewed, and recorded, or until an external blocker
requires user action. Do not notify the user just because one subagent or role
finished if the next role can proceed immediately.

For environment setup, convergence means the local toolchain can run the agreed
validation commands. The orchestrator should inspect the current state, repair
what is missing when allowed, and rerun the checks. If downloads, installers, or
unsandboxed commands require approval, request approval through the available
tooling. If setup cannot converge, record the exact missing tool, commands run,
and failure output in the active ExecPlan or `.agent/REPOSITORY_STATE.md`.

For implementation milestones, convergence means the implementation has passed
the appropriate validation ladder, reviewer blocking findings are resolved, and
the active ExecPlan records evidence. If the reviewer rejects a slice, route the
findings back to the implementation engineer, rerun focused validation, and
review again. Repeat until no blocking findings remain or the same blocker has
recurred enough times that further cycles would only churn.

Do not send a user-facing "done", "fixed", or "mostly done" update after a
nontrivial routing slice until harness verification has either converged or
recorded a concrete failure packet. A failure packet must include benchmark,
route index, net name, blocker set when available, changed error signature,
commands, artifact paths, attempted fixes, and the next safest role.

For visual-layout bug reports, convergence additionally requires either a full
GDS/JSON verification packet or an explicit statement that only a partial debug
artifact was produced. A partial debug-stop GDS can be useful evidence, but it
does not close a full-benchmark layout issue.

A failure handoff is acceptable only when the blocker is concrete: missing user
approval, missing external toolchain support, unavailable dependency/network,
or a repeated technical failure with enough evidence for the user or next agent
to decide. In that case, report the blocker, the attempts made, and the safest
next action.

Before every stop, send a user-facing chat update that says whether the current
objective converged, is blocked, or is being paused with known next steps. If a
question or approval is needed during implementation, ask immediately through
the available chat/tooling path so the user can receive a push notification when
their Codex/ChatGPT mobile settings allow it.

## Current Recommended Role Sequence

For `.agent/execplans/2026-07-10-crossing-verification-foundation.md`, use:

1. **Planner / Technical Lead**: decide what should be implemented next, map
   the current crossing, realization, port snapping, verification, and A* cost
   paths, and update the ExecPlan.
2. **QA / Harness Engineer**: design the test and evidence strategy before code
   changes: structured verification JSON, focused pytest assertions, and
   benchmark commands.
3. **Implementation Engineer**: execute the plan with scoped code changes and
   local tests.
4. **Reviewer**: independently inspect the implementation for bugs, missing
   tests, Python/Rust boundary issues, and accidental topology hints in
   `lidar-pure`.
5. **QA / Harness Engineer**: run the agreed verification chain and record
   focused tests plus `benes_4x4` / `benes_8x8` benchmark evidence.

Do not start `multiportmmi_8x8` heuristic tuning until the smaller benchmark
verification path produces actionable pass/fail evidence.

Do not port implementation from `baseline/lidar-pure-crossings` until the
Planner / Technical Lead has audited the relevant files and the QA / Harness
Engineer has defined how the port will be verified.

## Subagent Prompt Contract

Every subagent prompt should include this minimum contract:

    You are working in the TUMPhotonicRouter repository root. Do not assume a
    fixed absolute path; this checkout may be on Windows or Linux.
    Read AGENTS.md, .agent/PROJECT_GOAL.md, .agent/WORKFLOW.md, and the active
    ExecPlan before acting.
    Do not revert user or other-agent changes.
    Keep edits within the assigned file scope.
    Update the active ExecPlan if your task discovers durable facts or decisions.
    Report changed files, commands run, pass/fail results, and residual risks.

Add the relevant role brief from `.agent/roles/`.

## File Ownership Rules

When multiple agents work at once, assign file ownership:

- Planner/auditor normally reads only and edits the active ExecPlan.
- QA/harness engineer may edit tests, verification helpers, and benchmark
  scripts.
- Implementation engineer may edit the explicitly assigned source files.
- Reviewer should not edit files unless doing an approved fix pass.

For the current crossing verification milestone, avoid overlapping edits in:

- `translation/photonic_verification.py`
- `translation/route_rust_realization.py`
- `translation/route_rust_records.py`
- `translation/route_rust.py`
- `src/astar.rs`
- `src/py_router.rs`
- `tests/test_realized_crossing_verification.py`
- `tests/test_photonic_verification.py`

## Validation Expectations

For crossing work, the orchestrator should prefer this evidence chain:

1. Focused unit or fixture tests for geometry classification.
2. Structured JSON verification report under `build/verification/`.
3. Focused Rust tests if Rust route cost or crossing logic changed.
4. Focused Python tests around verification and realization.
5. `benes_4x4` and `benes_8x8` benchmark runs with exact commands and artifact
   paths.

Do not accept screenshots alone as proof.

When screenshots reveal a failure, convert them into structured evidence before
continuing substantial implementation: identify the produced artifact, route
indices/net names if possible, verification report counters, and the missing
fixture or GDS inspection needed to catch the issue next time.

## Workflow Retrospective Gate

After any long routing workflow, especially one that involved repeated
heuristic tuning or user-caught invalid geometry, activate QA / Harness
Engineer for a short retrospective before moving on. The harness retrospective
should answer:

- Did the validation packet actually prove the claim made to the user?
- Did the orchestrator use subagents or role passes at the escalation triggers?
- Which manual check should become a fixture, report field, benchmark lane, or
  reviewer rule?
- Which workflow files or ExecPlan notes need updates so the next agent does
  not repeat the same pattern?

Record durable process learnings in `.agent/WORKFLOW.md`, role briefs, the
active ExecPlan, or `.agent/REPOSITORY_STATE.md` as appropriate.

## Completion Rule

A milestone is complete only when:

- the active ExecPlan progress item is checked off with a timestamp;
- validation evidence is recorded;
- any new behavior is either covered by tests or explicitly listed as residual
  risk;
- blocking reviewer findings are resolved or explicitly downgraded with a
  rationale recorded in the active ExecPlan;
- git commit groups are coherent, or remaining dirty files are documented;
- the next recommended step is clear.
