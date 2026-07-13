# Git Workflow

This document defines how agents should use git in this repository. It is meant
to keep long-running agent work reviewable and recoverable.

## Commit Principles

Commits should be small enough to review and large enough to represent a useful
state. Do not create a commit for every tiny edit if the edits only make sense
together. Do not mix unrelated work in one commit.

Good commit boundaries:

- documentation/workflow setup;
- a focused test or harness addition;
- a single routing behavior change plus its tests;
- a mechanical refactor with no behavior change;
- benchmark data or artifact-generation support.

Bad commit boundaries:

- crossing search changes mixed with documentation cleanup;
- unrelated Python and Rust fixes with no shared acceptance criterion;
- generated artifacts mixed with source changes unless the artifact is the
  intended deliverable;
- “misc fixes” commits.

## Commit Message Format

Use a short imperative subject with a clear scope:

    docs: add agent orchestration workflow
    verification: emit crossing report JSON
    routing: separate physical loss from search penalties
    tests: cover bend-adjacent crossing rejection

When useful, add a body:

    Why:
    - what problem this commit solves

    Validation:
    - exact commands run
    - important pass/fail result

    Notes:
    - known residual risk or follow-up

## Before Committing

Before each commit:

1. Run `git status --short`.
2. Inspect the diff for the files being committed.
3. Ensure the commit does not include unrelated user or other-agent changes.
4. Run the narrowest meaningful validation for the change, or state why no test
   was run.
5. Update the active ExecPlan if the commit changes project facts, decisions,
   validation evidence, or next steps.

Use explicit pathspecs when staging. Avoid `git add .` in a dirty worktree
unless the whole dirty tree is intentionally part of the same commit.

## Repository State Summaries

Before every agent stop, pause, or handoff, update:

    .agent/REPOSITORY_STATE.md

The summary should include:

- branch and current commit;
- active ExecPlan;
- commit range summarized;
- what works now;
- what is still broken or unknown;
- validation commands and results;
- important generated artifacts;
- next recommended role/action.

This summary should not replace ExecPlan progress. It is a compact repository
state checkpoint for humans and future agents.

## Cleaning A Dirty Worktree

Do not clean a dirty worktree by reverting files unless the user explicitly asks
for that. In this repository, dirty files may contain user work or prior-agent
routing experiments.

To clean up before new implementation:

1. List dirty files with `git status --short`.
2. Group files by coherent commit intent.
3. Review each group with `git diff -- <paths>`.
4. Commit only groups whose intent and validation are understood.
5. Leave unknown or unrelated dirty files untouched and document them in
   `.agent/REPOSITORY_STATE.md`.

If a file group is too large to understand quickly, do not commit it just to
make the tree clean. First audit it or ask the user whether it should be parked,
committed, or ignored for the current task.

## Generated Artifacts

Generated files under `build/` are normally not committed. Commit generated
artifacts only when the user asks or when the artifact is a deliberate checked-in
baseline or fixture.

For benchmark evidence, record artifact paths in the ExecPlan and repository
state summary instead of committing large outputs by default.
