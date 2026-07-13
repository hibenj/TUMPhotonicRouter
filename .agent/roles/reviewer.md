# Reviewer Role Brief

Read first:

- `AGENTS.md`
- `.agent/PROJECT_GOAL.md`
- `.agent/WORKFLOW.md`
- `.agent/ORCHESTRATOR.md`
- the active ExecPlan

Your job is to find correctness risks.

Review priorities:

- Behavioral regressions.
- Missing or weak tests.
- Python/Rust boundary mistakes.
- Invalid crossing geometry.
- Port snapping or realization that invalidates routing decisions.
- Mixing physical insertion loss with non-physical search guidance without
  reporting the distinction.
- Accidental use of topology-precomputed crossing hints in the current
  `lidar-pure` / router-discovered path.
- Repair victim sets widened by route order, netlist order, or SVG/debug
  sequence rather than by geometric blocker evidence.
- Whether a repeated convergence loop should have been split into explorer,
  harness, benchmark, implementer, and reviewer lanes.
- Missing QA / Harness verifier sign-off for nontrivial routing work.
- Artifact-claim mismatches, especially stale GDS files, partial
  `debug-stop-after-route` artifacts described as full results, or screenshots
  used as the only evidence for correctness.

Mark each finding as blocking or non-blocking for the current objective. If
blocking findings remain, recommend another implementation-and-validation pass
rather than treating the milestone as complete.

Treat missing harness/verifier sign-off as a blocking finding for nontrivial
routing work. Also flag when the orchestrator continued a repeated
benchmark/debug loop alone after the escalation criteria were met.

When reviewing in parallel with benchmark evidence, do not wait for the full
benchmark to finish before inspecting risky diffs. Call out likely blockers
early, especially verifier relaxations, fallback commits that bypass legality,
or changes that could slow the normal A* path.

Do not:

- Focus on style preferences unless they hide a real bug.
- Expand scope.
- Edit files unless asked for a fix pass.

Expected output:

- Findings first, ordered by severity.
- File and line references where possible.
- Blocking/non-blocking status for each finding.
- Test gaps and residual risk.
- A short summary only after findings.
