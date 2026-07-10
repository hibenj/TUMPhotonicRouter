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
- `.agent/REPOSITORY_STATE.md`: compact handoff summary updated at least every
  10 commits and before major handoffs.
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

## Multi-Agent Roles

Use separate software-team roles when the task is large, ambiguous, or
correctness-sensitive. The roles may be performed by separate agents or by one
agent moving through the roles explicitly.

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

If the plan is wrong, the implementer should revise it and explain why instead
of silently diverging.

### Reviewer

The reviewer looks for defects, not style preferences. The reviewer should:

- Inspect diffs against the stated acceptance criteria.
- Prioritize behavioral bugs, regressions, missing tests, and unsafe assumptions.
- Check Python/Rust boundary changes carefully.
- Verify that debug artifacts and benchmarks prove the claimed behavior.
- Return concrete findings with file and line references.

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

4. Full local suites when shared behavior changed:

       cargo test
       PYTHONPATH=. .venv/bin/pytest -q

5. Benchmark or debug runs for routing behavior:

       BROWSER=/bin/true .venv/bin/python routing_flow.py <benchmark> <flags>

6. Physical-layout and artifact checks for crossing or PLM work:

       inspect generated build/routes/*.svg, build/crossings/*, and routed GDS
       record the exact files and any verification scripts used

Record which rung was run and what it proved. If a rung cannot be run, record
why and what risk remains.

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
