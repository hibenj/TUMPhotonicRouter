# Repository State

This file is a compact checkpoint for humans and future agents. Update it at
least every 10 commits and before major handoffs. It does not replace the active
ExecPlan.

## Current Snapshot

- Date: 2026-07-10
- Branch: `crossings/verification-foundation`
- Current clean-branch base: `731d0a9`
- Active ExecPlan:
  `.agent/execplans/2026-07-10-crossing-verification-foundation.md`

## Current Goal

Make TUMPhotonicRouter a very fast verified photonic router. The current phase
focuses on router-discovered optical crossings on `benes_4x4`, `benes_8x8`, and
then `multiportmmi_8x8`, with final-geometry verification and PDK/gdsfactory
crossing component realization.

## Worktree State

This worktree is intended to be the clean implementation path for crossing
verification and router-discovered crossing work. It should stay reviewable and
should not receive wholesale merges from the experimental branch.

At creation, this worktree was clean at `731d0a9`.

## Reference Branches

The branch `baseline/lidar-pure-crossings` contains a WIP prototype snapshot at:

    69ab9fd wip: snapshot experimental lidar-pure crossing prototype

Treat that branch as reference material only. It may contain useful ideas,
tests, benchmark imports, or implementation fragments, but it is not the clean
implementation path. Do not merge it wholesale into
`crossings/verification-foundation`.

If code is needed from the WIP branch:

1. Audit the relevant diff or file in the WIP branch.
2. Confirm the idea fits `.agent/PROJECT_GOAL.md` and the active ExecPlan.
3. Port the smallest useful piece manually or cherry-pick a narrow commit only
   after review.
4. Add focused verification before considering the port complete.

The old ExecPlan
`.agent/execplans/2026-07-06-match-lidar-multiportmmi-routing.md` is also
reference-only unless the user explicitly resumes it.

## Next Engineering Step

Activate the Planner / Technical Lead role from `.agent/ORCHESTRATOR.md` to
audit:

- crossing metadata creation;
- PDK/gdsfactory crossing component selection;
- crossing footprint to grid keepout/access conversion;
- geometry realization and crossing insertion;
- port snapping;
- final photonic verification;
- A* route-induced insertion-loss cost versus non-physical search penalties.
