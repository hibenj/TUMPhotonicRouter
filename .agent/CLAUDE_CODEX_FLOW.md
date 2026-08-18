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

## What Codex is reliable at, and what it is not

Codex is reliable when a task file gives it a fully specified target: exact
formulas, exact call-site shapes, exact expected values, a narrow file scope,
and clear criteria for reporting "this case is different" rather than forcing
it to fit the pattern. A task built this way (e.g. a test-recalibration task
that stated the sizing formula and worked the arithmetic per test) came back
correct on the first attempt, including correctly flagging the one case that
was not a simple recalibration instead of quietly forcing an assertion to
pass.

Codex is not reliable at recognizing and reusing a pre-existing architectural
mechanism it was not explicitly pointed at. When a change needs to compose
with an existing invariant or exemption rule living elsewhere in the
codebase, Codex tends not to go find it; instead it invents a new, parallel
mechanism to patch the symptom in front of it. In this repository's unify-
port-access-region work, two successive Codex attempts at the same
architectural migration produced a new bolt-on subtraction block that did not
exist in the first attempt and made the regression count worse, not better,
because neither attempt discovered the pre-existing exemption mechanism
(`_foreign_keepout_open_cells_for_spec`/`normal_port_runway_cells`) that
already solved the problem by construction. The fix, once Claude read that
mechanism directly, was much smaller than either Codex attempt.

Two concrete rules follow from this:

- Before dispatching a task that must compose with existing logic, Claude
  should find and name the relevant mechanism explicitly in the task file
  (file, function, and why it already handles part of the problem), not
  leave Codex to rediscover it. If Claude has not yet found that mechanism
  itself, that is a sign the planning step is not actually done yet.
- Watch for non-convergence across repeated attempts at the same
  architectural problem: if a second or third attempt is flat or worse (same
  or higher regression count, a new special-case block that was not present
  before), stop delegating that slice. Read the relevant code directly and
  implement it personally rather than continuing to iterate Codex on a
  problem that requires understanding why existing code is shaped the way it
  is.

## The diagnosis/implementation boundary

A real gap surfaced in this repository's own history: an investigation that
started from a wrong theory (a routing failure blamed on a fixed obstacle
margin) went through several rounds of writing a small, disposable Python
script, running it, reading the real output, and revising the theory --
entirely reasonable Explorer/Planner work -- and then, the moment the
correct, fully-specified fix became clear (one new ~25-line method, three
call sites, exact target values already worked out), Claude wrote that fix
directly instead of stopping to hand it to Codex. No rule was technically
broken by the diagnosis itself (writing and discarding probe scripts is not
"implementing a routing change," and PLANS.md/WORKFLOW.md both expect this
kind of iteration to stay with Claude), but there was no explicit moment
where the session asked "is this now a bounded, well-specified Implementation
Engineer task?" and acted on the answer. This section exists to make that
moment explicit instead of leaving it to habit.

Diagnosis stays with Claude, in full, no matter how much code it produces
along the way. Writing a monkeypatch script to dump internal state, running
it, discarding it, writing the next one -- this is reading and testing the
codebase, not implementing the fix, even though it involves an edit tool and
even though the loop can run many times in a row. Do not feel obligated to
hand any of this to Codex; round-tripping a five-second diagnostic script
through a full Codex dispatch (prompt assembly, sandboxed run, transcript
review) would slow the investigation down for no benefit, and Codex is not
the right tool for open-ended "what does this data actually show" work in
the first place.

The boundary is crossed the moment a diagnosis is stable enough to state as
a bounded change: an exact file and function, the exact edit, and (where
applicable) exact expected before/after values to check it against. At that
exact moment, stop and choose one of three paths, out loud, before writing
any of the shipped fix:

1. Write the Codex task file and dispatch it (the default -- this is what
   "Codex implements" means in practice: the point where the plan is
   finished, not the point where the code happens to get typed).
2. Apply the existing `WORKFLOW.md` "small localized edits" exception (a
   change so small that a Codex round-trip is pure overhead -- a one-line
   ExecPlan update, a typo, a doc fix) and say so.
3. Implement directly for a stated, recorded reason (Codex already failed on
   this exact slice at least once and the reason is on record, per the
   non-convergence rule above; or the fix is small but its correctness
   depends on interactive intermediate output from the diagnosis in a way
   that would not survive being restated as a static prompt). Record which
   of these applied in the ExecPlan's Decision Log -- "implemented directly
   because X" -- rather than silently defaulting to self-implementation.

Skipping this choice is itself the failure mode, not any one path being
wrong in a given case. A plan that goes straight from "diagnosis complete"
to "fix committed" with no visible decision about who implements it should
be treated as incomplete, the same way a milestone with no validation
evidence is treated as incomplete.
