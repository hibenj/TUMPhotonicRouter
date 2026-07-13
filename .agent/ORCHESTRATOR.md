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

For the current phase, the active ExecPlan is:

    .agent/execplans/2026-07-10-crossing-verification-foundation.md

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
