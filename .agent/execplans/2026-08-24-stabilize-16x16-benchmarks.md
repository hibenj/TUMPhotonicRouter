# Stabilize the 16x16-scale benchmarks

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. This document must be maintained in accordance with `.agent/PLANS.md`, which is checked into this repository at that path and describes the required structure and editing discipline for ExecPlans in general.

## Purpose / Big Picture

This repository, TUMPhotonicRouter, routes optical waveguides for photonic chips on a discrete grid (see `.agent/execplans/2026-08-24-modular-routing-strategies.md` for term definitions this plan reuses: obstacle map, ripup/repair, net, victim). Its benchmarks scale by port/net count: `benes_4x4` and `multiportmmi_8x8` are the fast, currently-clean smoke tests; `benes_16x16` and `multiportmmi_16x16` are the next scale tier; `multiportmmi_32x32` (known unstable, a slow/hanging route documented but not investigated) is the tier after that.

The repository owner's explicit direction (2026-08-24, via direct chat): before attempting any `multiportmmi_32x32`-scale work, get a genuinely clean, correct baseline through the 16x16 tier, and use whatever that investigation teaches about the router's current behavior to decide what "improvements" (config/defaults simplification, performance work, or repair-strategy work) actually mean -- not guess at that scope in advance.

Concretely, this plan closes two things: `benes_8x8`/`benes_16x16` were last confirmed clean 2026-08-18 and never re-checked despite substantial restructuring since; `multiportmmi_16x16` under its documented stable-baseline configuration fails with `RuntimeError: No route found for n_50`, a genuine, not-yet-root-caused repair-exhaustion problem, confirmed several times this session to be byte-identical and not caused by anything this session's own restructuring touched. After this plan, either `multiportmmi_16x16` routes cleanly under its stable-baseline config, or the exact geometric/repair reason it cannot is documented to the same depth as the already-resolved `n_70` finding was (see `.agent/REPOSITORY_STATE.md`'s Resolved Findings), with a recorded decision about whether fixing it is worth the risk/effort right now.

A person can see this working by running the exact benchmark commands in Concrete Steps and reading `build/verification/*.json` directly.

## Progress

- [x] (2026-08-24) Milestone 0: re-verified `benes_8x8` and `benes_16x16` are still clean after all restructuring since the 2026-08-18 baseline. `benes_8x8`: `error_count=0, warning_count=0` both verifications, 22.2s. `benes_16x16`: `error_count=0, warning_count=0` both verifications, 252.9s (~4.2 minutes) -- a concrete performance data point, not yet investigated further, ahead of any planned performance work.
- [ ] Milestone 1: root-cause `multiportmmi_16x16`'s `n_50` failure to the same depth as the resolved `n_70` finding -- direct geometric/repair evidence for why no legal arrangement exists among the involved nets, not just the error signature already on record.
- [ ] Milestone 2: decide, based on Milestone 1's findings, whether/how to fix it -- present options to the repository owner if a real design decision is needed, matching this repository's standing practice for repair-strategy changes.

## Surprises & Discoveries

(none yet beyond Milestone 0's clean re-verification, recorded in Progress above)

## Decision Log

(none yet)

## Outcomes & Retrospective

Not started.

## Context and Orientation

### What's already known about `n_50`

`multiportmmi_16x16` under its documented stable-baseline configuration (exact command in Concrete Steps) fails with:

    RuntimeError: No route found for n_50: mmi0_ps_array_0_heater_3,o2 -> mmi0_multiport_0_0,o4.
    source=(807, 193, 0), target=(933, 265, 0), allow_45_degree_turns=True.
    error=No repair route found; candidate_blockers=[49, 50];
    recent_errors=["repair_failed_net:net51:roundSome(4):rip[]:Illegal realized crossing:
    net 51 intersects net 50 at (1767.500, 1287.125) (not_perpendicular)",
    "lidar_pure_probe_commit:net51:roundSome(4):rip[]:Illegal realized crossing:
    net 51 intersects net 50 at (1805.500, 1287.125) (not_perpendicular)",
    "reroute_victims:net50:roundSome(1):rip[49, 50]:No route found",
    "reroute_victims:net50:roundSome(1):rip[49, 50]:No legal LiDAR crossing route found",
    "reroute_victims:net49:roundSome(1):rip[49, 50]:No route found",
    "reroute_victims:net49:roundSome(1):rip[49, 50]:No legal LiDAR crossing route found",
    "reroute_victims:net50:roundSome(1):rip[50]:No route found",
    "reroute_victims:net50:roundSome(1):rip[50]:No legal LiDAR crossing route found"]

This exact text has been confirmed byte-identical across at least 4 separate reproductions this session (before, during, and after every milestone of both plans completed today), and separately confirmed (2026-08-24, a dedicated check earlier this same session) to **not** share the specific bug the `n_70` fix closed (a string-prefix-recognition gap in victim-set expansion) -- the victim-set expansion mechanism is already working correctly here (`native_repair_keepout net=51 ripup=[49, 50]` is tried in both orderings), and every "Illegal ..." message in the trace already uses the recognized `"Illegal realized crossing"` prefix, never the unrecognized one. So this is a genuine, separate repair-exhaustion or geometric-congestion problem, not a parsing bug -- Milestone 1 must find out which, and why, the same way the `n_70` investigation did.

### The `n_70` investigation technique, to reuse here

`.agent/execplans/2026-08-20-ripup-repair-orchestration-restructuring.md`'s own Milestone 0 (see that plan's Surprises & Discoveries) root-caused the now-fixed `n_70` finding using: `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1` (an environment variable already wired into `src/py_router.rs`'s repair loop, printing `native_repair_*` trace lines to stderr showing every repair attempt, victim set, and outcome -- confirmed still present and working, used throughout this session's own `n_50` re-checks), and a direct geometric BFS connectivity-with-clearance probe (`_corridor_clearance_diagnostic`, referenced in that plan as producing `corridor_clearance_last_connected_radius`/`corridor_clearance_first_disconnected_radius`/`corridor_clearance_source_region_size`/`corridor_clearance_target_region_size`/`corridor_clearance_min_distance` fields in the per-net `build/routes/<benchmark>_<net>_FAILED.txt` diagnostic file) -- a check that is independent of the repair search itself, so it can distinguish "genuine geometric dead end" from "search strategy gap" the way a repair-trace log alone cannot. Milestone 1 must locate this diagnostic's actual current implementation (`grep -rn "corridor_clearance" src/*.rs translation/*.py`) and confirm it still works before relying on it, not assume it is unchanged since 2026-08-20.

### Toolchain and safety notes

Identical to both prior plans today -- `RUSTUP_TOOLCHAIN=stable-x86_64-unknown-linux-gnu PYO3_PYTHON="$PWD/.venv/bin/python"` override for `cargo`, `maturin develop --release` after any `src/*.rs` change. Do not run `--debug-svgs` with no value on `multiportmmi_8x8` or `multiportmmi_16x16`. `multiportmmi_16x16` under its stable-baseline config completes (or fails, as it currently does) in well under the 8-minute cap this session has used throughout; do not assume the much slower, killed `--attempt-diagnostics` variant an earlier session used -- that is a different, heavier flag this plan does not need.

## Plan of Work

**Milestone 1** reproduces `n_50` with `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1` and reads the full trace (not just the final error) to identify exactly which nets are involved and what every attempted repair strategy actually tried, then runs (or, if it no longer exists in this exact form, rebuilds a narrow equivalent of) the corridor-clearance BFS probe on the specific net(s) that end up blocked, to get direct geometric evidence: is there a real, physically-connected corridor between the relevant source/target pair at all, and if so, how much clearance-inflation does it survive before disconnecting. Record the full findings, with evidence, in Surprises & Discoveries -- matching the depth and citation style the `n_70` investigation used, not just a restatement of the existing error text.

**Milestone 2** is deliberately not pre-specified past "decide based on Milestone 1's findings, present options if a real design decision is needed" -- per `.agent/PLANS.md`'s guidance against over-specifying a design before the investigation that justifies it has happened, and matching this plan's own repository-owner direction to let Phase 1's findings, not advance guessing, decide what "improvements" mean next.

## Concrete Steps

From the repository root (`/home/benjamin/Documents/Repositories/working/TUMPhotonicRouter`, but do not assume this exact absolute path):

    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_8x8
    rm -rf build/routes build/verification
    PYTHONPATH=. .venv/bin/python routing_flow.py benes_16x16

    rm -rf build/routes build/verification
    PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT="0.05" PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES="90" PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG=1 \
    PYTHONPATH=. .venv/bin/python routing_flow.py multiportmmi_16x16 \
      --crossings true --crossing-mode lidar-pure --fanout-access-mode static-stubs \
      --routing-window-scale 0.35 --foreign-port-keepout-cells 0

The stable-baseline `multiportmmi_16x16` command above completes (with the `n_50` failure) in under 2 minutes as of this session's own measurements; cap any run at 8 minutes and treat a longer hang as its own failure signature to report, not silently retry.

## Validation and Acceptance

Milestone 0 (done): both `benes_8x8` and `benes_16x16` verification JSONs read `status=complete, success=true, error_count=0`.

Milestone 1: a recorded, evidence-backed root cause for `n_50`, citing direct diagnostic output (trace lines, BFS probe results, or both), not just the existing error text repeated again.

Milestone 2: either `multiportmmi_16x16` stable-baseline reads `success=true, error_count=0` after a fix, or a recorded decision (with the repository owner, if the fix is nontrivial) not to fix it now, with the reason stated.

## Idempotence and Recovery

Diagnostic runs are read-only with respect to the repository (they only write to gitignored `build/`). If a fix is implemented, validate it against the full ladder (`cargo test --lib`, `pytest -q`, `benes_4x4`, `multiportmmi_8x8` both configs, `benes_8x8`/`benes_16x16`, `multiportmmi_16x16` stable-baseline) before considering it done, per this repository's established discipline this session.

## Artifacts and Notes

(to be filled in as Milestone 1 produces diagnostic output worth preserving)

## Interfaces and Dependencies

None prescribed yet -- Milestone 1 is read-only investigation; Milestone 2's scope depends entirely on what it finds.
