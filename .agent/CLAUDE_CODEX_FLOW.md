# Claude + Codex Flow

This document defines how this repository's existing agent workflow
(`.agent/WORKFLOW.md`, `.agent/ORCHESTRATOR.md`, `.agent/roles/`,
`.agent/PLANS.md`) maps onto a session where a Claude agent (in Claude Code)
does the reasoning and Codex CLI does the coding. It does not replace those
documents; it says which role each tool plays and how they hand off work.

The rest of `.agent/` was already written around the same three references
this split is built on (see the Source Notes section of `WORKFLOW.md`):
self-contained living ExecPlans, repository-local knowledge as the system of
record, and scaling agent work around task objectives and validation packets
rather than ad-hoc supervision. This file operationalizes that for two tools
instead of one.

## Division of labor

Claude is the reasoning side. In this flow, Claude plays every role in
`.agent/WORKFLOW.md` and `.agent/ORCHESTRATOR.md` except Implementation
Engineer: Orchestrator, Planner / Technical Lead, Explorer / Codebase Audit,
Reviewer, and QA / Harness Engineer. Claude reads code, decides what to build,
writes and maintains the active ExecPlan, designs verification strategy,
reviews diffs, and decides pass/fail.

Codex is the coding side. Codex CLI (`codex exec`, invoked non-interactively
through `.agent/scripts/codex_task.sh`) plays only Implementation Engineer:
it receives a bounded task with an explicit file scope and acceptance
criteria, edits the working tree, runs the validation commands the task asks
for, and reports back. Codex does not decide what the project should do next
and does not review its own work as the Reviewer or QA / Harness role.

Claude must not silently do the coding itself as a shortcut. If a change is
small enough that spinning up Codex would be pure overhead (a one-line
ExecPlan update, a typo fix, editing `.agent/` docs), Claude may edit directly
under the existing "small localized edits" exception in `WORKFLOW.md`. Any
change to routing behavior, obstacle handling, crossing logic, path-length
matching, electrical routing, benchmark definitions, or other source under
`translation/`, `src/`, `routing_flow*.py`, or `python/photonic_router/`
should go through Codex as the Implementation Engineer, the same way it would
go through a human implementer in the existing role split.

## Invoking Codex

Use the wrapper script rather than calling `codex exec` by hand, so every run
is logged the same way:

    .agent/scripts/codex_task.sh <slug> <task-file> [--sandbox MODE] [--model MODEL]

`<task-file>` is a Markdown file Claude writes before the call (a scratch file
is fine; it does not need to live in the repository). It should be a bounded
Implementation Engineer task in the shape the orchestrator role brief already
describes in `ORCHESTRATOR.md`'s "Subagent Prompt Contract": which files
Codex must read, which files it may edit, which files it must not touch, the
exact expected output, and the validation commands to run before reporting
back. The script itself prepends the standard subagent contract and the full
text of `.agent/roles/implementer.md`, so the task file only needs to state
the task-specific parts.

Defaults:

- Sandbox: `workspace-write`. Codex can edit files anywhere under the repo
  checkout but cannot reach the network or write outside the repo. This is
  the right default for Rust/Python edits, `cargo check`, `cargo test`,
  `pytest`, and running `routing_flow.py` benchmarks, since the toolchain and
  crate/package caches are already local. Pass `--sandbox read-only` for an
  audit-only task (mirrors the Explorer role) that must not touch files, and
  reserve `danger-full-access` for a task that genuinely needs network access
  (for example, restoring a `cargo`/`pip` cache from scratch); confirm with
  the user before using it, since it removes Codex's filesystem and network
  containment.
- Approval policy: `never` (`-c approval_policy=never`). There is no
  interactive terminal for Codex to prompt in this flow, so an approval-gated
  action would otherwise hang the run. This means Codex will not ask
  permission mid-task; keep tasks scoped tightly enough that this is safe,
  and prefer `read-only` sandbox for anything Claude is not ready to have
  edited.
- Model: Codex's own configured default. Only pass `--model` when the task
  has a specific documented reason, consistent with `ORCHESTRATOR.md`'s
  "Model And Reasoning Policy" (do not downgrade correctness-critical work
  for cost or latency).

Each run writes `.agent/codex_runs/<UTC timestamp>-<slug>/` containing:

- `prompt.md`: the exact prompt Codex received (contract + role brief + task).
- `transcript.jsonl`: the full `--json` event stream from the run.
- `last-message.txt`: Codex's final report, in the "Expected output" shape
  from `.agent/roles/implementer.md`.
- `exit_code.txt`: the `codex exec` process exit code (nonzero means the
  Codex run itself crashed or was rejected, not that Codex disagreed with the
  task; read `last-message.txt` regardless).

`.agent/codex_runs/` is git-ignored except for a `.gitkeep`; these are working
logs, not durable project knowledge. If a run surfaces a fact or decision that
future agents need, copy it into the active ExecPlan or
`.agent/REPOSITORY_STATE.md` the same way any other subagent finding would be
recorded per `ORCHESTRATOR.md`.

## After a Codex run

Claude does not treat a Codex run as complete just because the process
exited 0 or the transcript looks confident, for the same reason
`ORCHESTRATOR.md` says subagent output is never automatically correct:

1. Read `last-message.txt` for the changed-files/behavior/tests/risks report.
2. Run `git status --short` and `git diff` over the claimed file scope.
   Confirm Codex stayed inside its assigned scope and did not touch the
   protected files listed in `ORCHESTRATOR.md`'s "File Ownership Rules"
   unless it was explicitly assigned there.
3. Act as Reviewer per `.agent/roles/reviewer.md` against the diff.
4. For nontrivial routing changes, act as QA / Harness Engineer per
   `.agent/roles/harness.md` and run the validation ladder in
   `WORKFLOW.md`; do not accept the Codex run's own claimed test results as
   the verifier packet.
5. Update the active ExecPlan's `Progress`, `Surprises & Discoveries`, and
   `Decision Log` sections, and update `.agent/REPOSITORY_STATE.md` before
   any stop or handoff, per `.agent/GIT_WORKFLOW.md`.
6. If review or harness verification finds blocking issues, write a new task
   file describing the fix and call `.agent/scripts/codex_task.sh` again.
   Keep iterating through implementation and validation, per the convergence
   policy in `WORKFLOW.md`, rather than declaring the slice done with known
   blocking findings.

## Committing

Codex's `workspace-write` sandbox can run `git` inside the repo, but in this
flow Codex should not be the one deciding commit boundaries. Ask Codex to
leave changes uncommitted (or stage them, at most) and have Claude review the
diff, follow `.agent/GIT_WORKFLOW.md`, and create the actual commit after
review, the same way Claude would review and commit its own edits. State this
explicitly in the task file when it matters (for example, a task that
naturally ends with a passing test suite and an obvious commit point).

## When not to use this flow

Planning, architecture decisions, review, and verification-strategy design
stay with Claude even when they are long or effortful; delegating those to
Codex would collapse the reasoning/coding split this document exists to keep.
Conversely, do not hand Codex a vague objective ("fix the crossing bug") and
expect it to plan its own work; that is the Planner role's job, and it stays
with Claude. Write the task file the way `ORCHESTRATOR.md` asks any role
brief to be written for a subagent: concrete, bounded, with an explicit file
scope.
