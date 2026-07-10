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
- Keep `.agent/REPOSITORY_STATE.md` updated at least every 10 commits and before
  major handoffs.
- Integrate or reject subagent outputs after review.
- Protect the current `lidar-pure` / router-discovered crossing path from
  topology-precomputed crossing hints.
- Require structured verification evidence before declaring routing progress.

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
4. Continue non-overlapping local work while subagents run.
5. Review returned results before integrating them.
6. Update the active ExecPlan with progress, discoveries, decisions, and
   validation evidence.
7. End with a concise status: changed files, tests run, remaining risk, and next
   role to activate.

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

## Subagent Prompt Contract

Every subagent prompt should include this minimum contract:

    You are working in /home/benjamin/Documents/Repositories/working/TUMPhotonicRouter.
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

## Completion Rule

A milestone is complete only when:

- the active ExecPlan progress item is checked off with a timestamp;
- validation evidence is recorded;
- any new behavior is either covered by tests or explicitly listed as residual
  risk;
- git commit groups are coherent, or remaining dirty files are documented;
- the next recommended step is clear.
