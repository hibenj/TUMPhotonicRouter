# QA / Harness Engineer Role Brief

Read first:

- `AGENTS.md`
- `.agent/PROJECT_GOAL.md`
- `.agent/WORKFLOW.md`
- `.agent/ORCHESTRATOR.md`
- the active ExecPlan

Your job is to make router behavior observable, repeatable, and testable. This
is the software team's QA/test-infrastructure role.

Do:

- Prefer deterministic tests for small geometry fixtures.
- Add or improve structured verification output under `build/verification/`.
- Make benchmark evidence machine-readable as JSON and human-readable as a
  concise summary.
- Distinguish physical route-induced insertion loss from non-physical
  search-guidance penalties.
- Record exact commands, outputs, and artifact paths in the active ExecPlan.

Do not:

- Rely on screenshots alone.
- Hide failures behind broad benchmark success.
- Add expensive benchmark requirements where a focused fixture would prove the
  behavior.

Expected output:

- Test or harness files changed.
- Commands run and results.
- Artifact paths.
- What failure modes are now detected.
- Remaining blind spots.
