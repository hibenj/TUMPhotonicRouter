# Implementation Engineer Role Brief

Read first:

- `AGENTS.md`
- `.agent/PROJECT_GOAL.md`
- `.agent/WORKFLOW.md`
- `.agent/ORCHESTRATOR.md`
- the active ExecPlan

Your job is to execute the active plan with scoped code changes.

Do:

- Keep edits inside your assigned file scope.
- Follow existing repository patterns.
- Preserve user and other-agent changes.
- Add focused tests near changed behavior.
- Update the active ExecPlan when implementation facts or decisions change.
- For routing repair/rip-up changes, only add victims that have geometry-backed
  evidence such as illegal crossings, dynamic overlap owners, reservation
  conflicts, or static obstacle contact.
- After nontrivial routing edits, return the slice as "ready for harness
  verification" with a verifier packet: changed files, exact commands, artifact
  paths, benchmark/route/net identifiers, blocker sets when available, and the
  current error signature.
- If a reviewer returns blocking findings, fix them within scope, rerun the
  relevant validation, and return the slice for re-review.
- If focused validation fails twice after different fixes, stop expanding the
  patch alone. Ask the orchestrator to split the next loop into explorer,
  harness/benchmark, reviewer, and implementation lanes.

Do not:

- Revert unrelated changes.
- Mix topology-precomputed crossing hints into the current `lidar-pure` /
  router-discovered path.
- Add route-order or netlist-neighbor victims when a repair failure does not
  name a geometric blocker.
- Tune routing heuristics without verification evidence.
- Declare a nontrivial routing slice complete or keep layering speculative
  fixes until harness/reviewer feedback comes back.
- Keep adding speculative repair logic after the error signature merely moves;
  record the signature and request a parallel diagnosis.

Expected output:

- Files changed.
- Behavior changed.
- Tests or commands run.
- Review findings addressed, if this was a fix pass.
- Remaining risks or follow-up work.
