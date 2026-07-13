# Agentic Coding Workflow

This document defines the repository workflow for agent-assisted coding in
TUMPhotonicRouter. It complements `AGENTS.md`, which explains the codebase, and
`.agent/PLANS.md`, which defines the format for executable plans.

## Purpose

The goal is to make agent work repeatable, reviewable, and restartable. A future
agent should be able to enter the repository, read the stable instructions and
the active task plan, and continue without relying on hidden chat history.

This workflow is intentionally lightweight. It should improve engineering
discipline without turning every small edit into process work.

The guiding principle is agent legibility: knowledge that matters to future work
must be visible in the repository. If an architectural decision, benchmark
result, recurring failure, or validation command only lives in chat, it is not
available to the next agent.

## Repository Structure

Use these locations consistently:

- `AGENTS.md`: stable project guide and architecture orientation.
- `.agent/PROJECT_GOAL.md`: stable repository-level goal and current focus.
- `.agent/WORKFLOW.md`: stable collaboration workflow for agents.
- `.agent/ORCHESTRATOR.md`: lead-agent guide for coordinating subagents or
  sequential role passes.
- `.agent/roles/`: reusable role briefs for planner, implementer, reviewer, and
  harness work.
- `.agent/GIT_WORKFLOW.md`: commit, staging, and repository-state checkpoint
  rules.
- `.agent/REPOSITORY_STATE.md`: compact handoff summary updated before every
  agent stop, pause, or handoff.
- `.agent/PLANS.md`: policy and template for self-contained ExecPlans.
- `.agent/execplans/`: living task plans for complex features, refactors, and
  debugging efforts.
- `.agent/tasks/`: optional future task registry for orchestration. Do not add
  this directory until there is an actual need to coordinate multiple concurrent
  tasks.
- `docs/`: user-facing or research-facing design notes and reports.
- `Agent_implementation_files/`: historical implementation notes. Treat these
  as background material, not active process policy.

Keep stable guidance small and discoverable. Put task-specific detail in the
active ExecPlan, not in `AGENTS.md` or this workflow document.

## When To Use An ExecPlan

Create or update an ExecPlan when a task has any of these properties:

- It touches both Rust and Python.
- It changes routing behavior, obstacle handling, crossing logic, path-length
  matching, electrical routing, benchmark definitions, or generated artifacts.
- It requires benchmark evidence, performance comparison, or multi-step
  debugging.
- It is likely to be paused and resumed by a different agent.
- It includes meaningful uncertainty that should be resolved through prototypes
  or staged validation.

Do not create an ExecPlan for small localized edits, typo fixes, or a targeted
test adjustment unless the user asks for one.

An ExecPlan must be self-contained. It should define the user-visible outcome,
the relevant repository context, the exact commands to run, the acceptance
criteria, and the current state of progress. A stateless agent or a human novice
should be able to complete the task from the plan and the working tree alone.

## Standard Single-Agent Loop

For ordinary implementation work, use this loop:

1. Orient by reading `AGENTS.md`, the active ExecPlan if one exists, and the
   relevant source files.
2. State the immediate implementation plan briefly before editing.
3. Make scoped edits.
4. Run focused validation near the changed behavior.
5. Broaden validation when the change affects shared routing behavior.
6. Summarize what changed, what was validated, and what remains risky.

Keep the working tree safe. Never revert user changes unless explicitly asked.
If unrelated files are dirty, leave them alone.

## Convergence And Notification Policy

When the user gives an objective, agents should work it to convergence instead
of repeatedly asking for the next step. Convergence means the requested setup,
implementation slice, or milestone has passed the highest validation rung that
is practical for its risk, and any blocking review findings have been resolved.

For setup work, continue through detection, repair, and validation until the
toolchain or local environment actually works. If installation, downloads, or
unsandboxed commands require user approval, request that approval through the
available tooling and then continue after approval. If the same blocker remains
after concrete retry attempts, record the exact blocker, commands, and observed
failures in the active ExecPlan or repository state, then notify the user.

For implementation work, do not stop after a first patch when the next local
step is obvious. Run the focused validation, address failures, and repeat until
the slice passes or the blocker is external. If a reviewer finds blocking
issues, send the work back through implementation and validation, then review
again. Repeat that loop until there are no blocking findings or until the same
class of blocker has repeated enough times that continuing would only churn.

Convergence does not mean one agent should personally perform every loop in one
long serial prompt. Escalate from single-agent work to a parallel bundle when a
focused benchmark or route stop fails twice after different hypotheses, when a
failure moves but keeps the same root shape, or when the next work can be split
cleanly into harness, code audit, implementation, benchmark evidence, and
review. In that situation, the orchestrator should assign bounded lanes with
disjoint file ownership and integrate their outputs, rather than continuing to
invent and test every hypothesis in the main window.

### Routing Verification Gate

Nontrivial routing work cannot be self-certified by the implementer or
orchestrator. A QA / Harness role must inspect the structured reports and
relevant generated SVG/GDS/debug artifacts and record a verdict before the user
is told that the slice is fixed or complete. Valid verdicts are `PASS`, `FAIL`,
`BLOCKED`, and `INCONCLUSIVE`.

This gate applies to changes in routing behavior, crossing legality,
realization, verification semantics, Python/Rust boundaries, A* repair logic,
benchmark-specific routing order, or any multi-file routing state. The only
exception is when user approval, credentials, a missing local tool, or another
external blocker must be surfaced immediately.

The verifier packet must include changed files, exact commands, artifact paths,
benchmark/route/net identifiers, blocker sets where available, and the changed
error signature. If a visual screenshot reveals invalid geometry, the harness
role must classify it as a validation blind spot until there is a deterministic
report, fixture, or GDS/SVG inspection path that would catch it.

For routing repair and rip-up, route order is not physical evidence. Candidate
blockers and victim sets must be backed by geometry: illegal crossing
participants, dynamic overlap owners, crossing-reservation conflicts, static
obstacle contacts, or a deterministic fixture/report that names the net. Do not
add neighboring routes merely because they are adjacent in the netlist, native
job order, or debug SVG sequence. If a `No route found` retry lacks a geometric
owner, report it as an incomplete repair signal or add harness visibility
before widening the victim set.

For router-discovered crossings, A* should reject known-illegal crossing moves
during neighbor exploration whenever the active state has enough information
about partner geometry, perpendicularity, margins, footprint clearance, and
crossing order. Post-route realized crossing validation is still required, but
it is a safety net for discretization/realization gaps, not the primary place
to discover avoidable illegal moves.

Use judgment about scope. Small localized edits may converge with a focused
test or syntax check. Behavior-changing router work, crossing realization,
Python/Rust boundary changes, and performance-sensitive A* changes should
converge through the multi-agent roles and the validation ladder before being
claimed complete.

Notify the user every time the chat/agent run stops, even if the stop is only a
pause between implementation slices. Also notify immediately when approval,
clarification, credentials, a missing local tool, or any other user action is
required. Do not wait for a commit batch or a 10-commit checkpoint before
surfacing questions.

Phone push delivery is an app and operating-system concern, not something the
repository can guarantee. The repository policy is to send a normal Codex chat
message at every stop or required-question point; mobile push notifications
depend on the user's Codex/ChatGPT notification settings, device permissions,
and whether the product currently forwards that kind of thread update to the
phone.

## Multi-Agent Roles

Use separate software-team roles when the task is large, ambiguous, or
correctness-sensitive. The roles may be performed by separate agents or by one
agent moving through the roles explicitly.

When roles run in parallel, keep their ownership narrow:

- Planner / Technical Lead owns the hypothesis map and next decision.
- Explorer / Codebase Audit owns read-only source tracing and baseline
  comparison.
- QA / Harness Engineer owns fixtures, reports, and repeatable evidence.
- Implementation Engineer owns the smallest assigned source edit.
- Benchmark / Evidence work, usually under QA / Harness, owns repeated command
  execution and concise error-signature reporting.
- Reviewer owns correctness review of risky diffs and evidence before the next
  implementation loop.

The orchestrator should not wait for all possible lanes before acting on clear
evidence, but it should prevent overlapping edits and record durable findings in
the active ExecPlan.

### Planner / Technical Lead

The planner converts the product/research goal into an executable technical
plan. The planner should:

- Read enough code to understand current behavior.
- Define the user-visible outcome and acceptance criteria.
- Break work into independently verifiable milestones.
- Identify validation commands and expected evidence.
- Record decisions and assumptions in the ExecPlan.

The planner should avoid over-specifying low-level implementation details before
reading the code. A good plan says what must be true and where likely changes
belong; it does not lock the implementer into stale guesses.

### Implementation Engineer

The implementation engineer follows and updates the ExecPlan. The implementer
should:

- Re-read the relevant plan sections before editing.
- Keep changes small enough to review.
- Update `Progress`, `Surprises & Discoveries`, and `Decision Log` as facts
  change.
- Add or adjust tests close to the changed behavior.
- Prefer repository patterns over new abstractions.
- Address blocking reviewer findings and rerun the relevant validation before
  returning the slice as complete.

If the plan is wrong, the implementer should revise it and explain why instead
of silently diverging.

### Reviewer

The reviewer looks for defects, not style preferences. The reviewer should:

- Inspect diffs against the stated acceptance criteria.
- Prioritize behavioral bugs, regressions, missing tests, and unsafe assumptions.
- Check Python/Rust boundary changes carefully.
- Verify that debug artifacts and benchmarks prove the claimed behavior.
- Return concrete findings with file and line references.
- Mark findings as blocking or non-blocking for the current objective.

The reviewer should not implement new scope during review unless the user asks
for a fix pass.

### QA / Harness Engineer

The QA/harness engineer owns repeatable evidence. This role is the software
team's test and validation engineer. In this repository, "harness" means tests,
benchmark commands, debug traces, structured JSON verification reports,
generated SVG/GDS diagnostics, and scripts that make router behavior observable.

The harness engineer should:

- Identify the narrowest command that proves a behavior.
- Add regression tests when a failure can be made deterministic.
- Preserve benchmark commands and important output snippets in the ExecPlan.
- Avoid relying only on visual screenshots for routing correctness.
- Distinguish compile success, unit success, integration success, benchmark
  success, and physical-layout verification success.
- Turn repeated manual checks into scripts or tests when they become part of the
  normal routing workflow.

Harness work is not optional for routing changes. A router change without
observable evidence is unfinished. This role makes sense here because photonic
routing can appear to work visually while still producing invalid geometry; the
QA/harness engineer turns those checks into deterministic evidence.

### Explorer / Codebase Audit

The explorer answers bounded architecture questions without owning behavior
changes. This role is useful when the main agent is stuck in a hypothesis loop
or when a reference branch may contain ideas that must be audited before use.

The explorer should:

- Read the requested files and trace the relevant data/control flow.
- Compare a reference branch or historical plan only when assigned.
- Identify candidate root causes, invariants, and risky coupling points.
- Report exact files/functions read and the smallest recommended next test or
  implementation slice.
- Edit only the active ExecPlan or repository state when asked to record durable
  findings.

The explorer should not:

- Make source changes.
- Run broad benchmark loops unless assigned as the benchmark lane.
- Port code from reference branches without planner and reviewer approval.

### Benchmark / Evidence Lane

The benchmark lane is a narrow harness duty for repeated commands. It should
not redesign the implementation while a command runs. Its output should be
small and comparable between runs:

- exact command and environment assumptions;
- pass/fail;
- total time and important timing buckets;
- route index, net name, and blocker set for failures;
- changed error signature compared with the previous run;
- artifact paths, especially structured JSON under `build/verification/`.

Use this lane when expensive route stops or benchmarks would otherwise block
all other reasoning in the main agent window.

## Recommended Multi-Agent Sequence

For substantial work, use this sequence:

1. Planner / technical lead drafts or refreshes the ExecPlan.
2. Reviewer performs a plan review and calls out missing acceptance criteria,
   weak assumptions, or unsafe scope.
3. Implementation engineer completes the next milestone and updates the
   ExecPlan.
4. QA/harness engineer runs or improves validation and records evidence.
5. Reviewer reviews the diff and evidence.
6. Implementation engineer fixes review findings.
7. Planner or implementation engineer writes the retrospective when the
   milestone is done.

This sequence can be shortened for small work, but do not skip the review and
validation mindset for changes to routing correctness.

If the reviewer returns blocking findings, repeat steps 4 through 6 until the
blocking findings are resolved or the task is genuinely blocked. Do not report a
milestone as done while known blocking review findings remain.

## Objective-Based Orchestration

For now, this repository does not need a full external orchestrator. When the
work grows beyond one interactive session, use task-level objectives rather than
micromanaging sessions. A good task has:

- A clear deliverable, such as "stabilize legal crossing verification on
  `benes_16x16`" rather than "edit `src/astar.rs`."
- A single owner role for the current step.
- Explicit dependencies on other tasks or milestones.
- A validation packet: commands, artifacts, and reviewer notes that prove the
  outcome.
- A follow-up list for useful discoveries that are outside the current scope.

If orchestration becomes necessary, add a small task registry under
`.agent/tasks/` before introducing any automation. Each task should point to one
ExecPlan or state why no ExecPlan is needed. The first automation target should
be status visibility and stale-plan detection, not automatic code changes.

## Validation Ladder

Use a validation ladder instead of jumping directly to expensive benchmarks.
Choose the highest rung justified by the change.

1. Formatting and build checks:

       cargo fmt
       cargo check

2. Focused Rust tests:

       cargo test <module_or_test_name>

3. Focused Python tests:

       PYTHONPATH=. .venv/bin/pytest -q tests/<test_file>.py

   Windows PowerShell equivalent:

       $env:PYTHONPATH='.'; .\.venv\Scripts\pytest.exe -q tests\<test_file>.py

4. Full local suites when shared behavior changed:

       cargo test
       PYTHONPATH=. .venv/bin/pytest -q

   Windows PowerShell equivalent:

       cargo test
       $env:PYTHONPATH='.'; .\.venv\Scripts\pytest.exe -q

5. Benchmark or debug runs for routing behavior:

       BROWSER=/bin/true .venv/bin/python routing_flow.py <benchmark> <flags>

   Windows PowerShell equivalent:

       .\.venv\Scripts\python.exe routing_flow.py <benchmark> <flags>

   On Windows, `routing_flow.py` currently opens debug SVGs with the default
   browser when debug SVG generation is enabled. Record that behavior if it
   affects a validation run.

6. Physical-layout and artifact checks for crossing or PLM work:

       inspect generated build/routes/*.svg, build/crossings/*, and routed GDS
       record the exact files and any verification scripts used

Record which rung was run and what it proved. If a rung cannot be run, record
why and what risk remains.

## Command Runtime Guardrails

No agent should leave a build, test, benchmark, or diagnostic command running
without a stated time budget. Before starting an expensive command, say or
record what it is expected to prove, the local cap, and what will happen if it
does not finish. For normal focused routing validation, use a 5 minute cap by
default. Use a longer cap only when the command is intentionally a full
benchmark or the user explicitly approves it.

Poll long commands regularly. If the command exceeds the cap, stop only the
matching repository process, preserve any useful partial output or artifacts,
and record the timeout as part of the failure signature. Do not treat an
overnight or unattended process as convergence.

After an interrupted, resumed, or compacted session, first check for stale
repository processes such as `routing_flow.py`, `pytest`, `cargo`, `maturin`,
or `rustc`. Clean up stale validation/build processes before starting new
work. This check is part of the stop/resume protocol, not optional cleanup.

## Plan Granularity

Avoid plans that are too low-level too early. Early plans should define:

- The behavior to enable or fix.
- The files and subsystems likely involved.
- The evidence required to prove success.
- The risks and unknowns that need investigation.

Only add detailed implementation steps after source inspection or a prototype
has made the design clear. If a task starts as research, make the first
milestone a proof of feasibility with observable output.

## Documentation Hygiene

Agent-generated repositories drift when stale patterns remain visible. Keep the
agent knowledge base clean:

- Mark old ExecPlans as completed, superseded, or abandoned in their
  retrospective.
- Move durable lessons from completed plans into `AGENTS.md`, this workflow, or
  focused docs only when they apply beyond one task.
- Remove or rewrite instructions that disagree with current code behavior.
- Prefer one canonical command over several near-duplicates.
- Encode recurring review feedback as tests, scripts, or documented rules.
- When a routing loop reveals a reusable process failure, such as
  over-centralized orchestration, missing artifact inspection, repeated moving
  failure signatures, stale/partial artifact confusion, or weak validation
  packets, record the task-specific evidence in the active ExecPlan and promote
  the general lesson into `.agent/WORKFLOW.md`, `.agent/ORCHESTRATOR.md`, or
  the relevant role brief before stopping.

A periodic doc-gardening pass should inspect `AGENTS.md`, `.agent/WORKFLOW.md`,
`.agent/PLANS.md`, active ExecPlans, and high-traffic docs for contradictions.

## Handoff Rules

Before handing off substantial work, update the active ExecPlan so it includes:

- Current git status summary.
- Completed and remaining progress items.
- Commands run and whether they passed.
- Generated artifacts that matter.
- Known blockers or risks.
- The exact next recommended step.

The next agent should not need chat history to continue.

## Workflow Health Checks

Periodically ask these questions:

- Is `AGENTS.md` still stable guidance, or has task-specific detail leaked in?
- Is the active ExecPlan self-contained enough for a novice?
- Are old plans clearly historical, completed, or superseded?
- Are benchmark claims backed by commands and artifacts?
- Is the validation cost proportional to the risk of the change?
- Has a repeated agent mistake been fed back into docs, tests, or harnesses?

If the answer is no, fix the documentation structure before adding more routing
logic.

## Source Notes

This workflow was shaped by three OpenAI references:

- `https://developers.openai.com/cookbook/articles/codex_exec_plans`: use
  self-contained living ExecPlans for long-running work.
- `https://openai.com/index/harness-engineering/`: make repository-local
  knowledge the system of record and invest in feedback loops that make agent
  work legible.
- `https://openai.com/index/open-source-codex-orchestration-symphony/`: scale
  agent work around task objectives, validation packets, and documented workflow
  rather than ad-hoc session supervision.

These source notes are background. Future agents must not depend on reading
external pages to follow this repository workflow.
