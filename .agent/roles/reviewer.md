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

Do not:

- Focus on style preferences unless they hide a real bug.
- Expand scope.
- Edit files unless asked for a fix pass.

Expected output:

- Findings first, ordered by severity.
- File and line references where possible.
- Test gaps and residual risk.
- A short summary only after findings.
