# First search attempt at the retry budget

Status: not adopted (owner decision 2026-11-05; the switch left the main
tree 2026-11-09). Base commit of the patch: `45c9aa2` (the patch is the
commit `bc2c0d8` without its `CHANGELOG.md` hunk; `docs/CONFIGURATION.md`
and `_rust.pyi` are regenerated, not patched).

## Idea

Speed pass B1. On the heavy cells every failing first attempt dies exactly
at the 2 M expansion budget after three to four windows (7 to 8 s each) and
the 10 M retry then succeeds (Benes 32x32 lidar-pure: 51 of 51 retries). The
failing first attempts burn 20 to 60 % of all expansions. Running the first
attempt directly at the retry budget saves that time, but it is not exact:
the per-window iteration cap is `min(max_iterations, remaining budget)`, so
a larger budget can let an earlier, smaller window succeed with a different
route (gate: Benes 4x4 lidar-pure +11.7 um). Hence a switch,
`NegotiationConfig.first_attempt_full_budget` /
`PHOTONIC_ROUTER_NEGOTIATED_FIRST_ATTEMPT_FULL_BUDGET=1`, in
`engine/negotiation/loop_.rs`, and the 27-cell table.

## Measurement

Two 27-cell tables at `bc2c0d8` on cda-server-4, 2026-11-05, partly
concurrent with each other and with other users' jobs:
`tables/reference.json` (switch off) and
`tables/first_attempt_full_budget.json` (switch on), rendered in
`tables/table.md`.

Crossings identical on all 27 cells; six lengths move by -0.10 to +0.13 %
(Benes 4x4 lidar-pure, 16x16 contribution 1, 32x32 contribution 1, ADEPT
16x16 lidar-pure, 64x64 lidar-pure and contribution 1); wall: Benes 32x32
lidar-pure 811 to 629 s (0.78), Benes 32x32 contribution 1 0.79, ADEPT 32x32
lidar-pure 0.82, but ADEPT 64x64 lidar-pure 2345 to 2719 s (1.16) and Benes
16x16 contribution 1 29 to 54 s (1.86); total over the 27 cells 0.99.

## Result and decision

A wash: the time saved on the Benes retries is spent on the tail of failing
10 M first attempts on the dense cells, and six paper lengths move. Not
adopted. The lever it pointed at, the failing attempts, was then taken by
the window ladder (`../window-ladder/`): with window 0 only, a failing
attempt has no tail, which is why `window0_full_budget` in that record is
this switch combined with the ladder (0.88 total, but the same six length
moves and the Benes 16x16 contribution 1 regression). The ablation variant
`first_attempt_full_budget` in `scripts/results/ablation.sh` went with the
switch.

## Reproduce

```bash
git checkout 45c9aa2
git am experiments/negotiation-first-attempt-full-budget/patches/*.patch
.venv/bin/maturin develop --release
RESULTS_ROOT=$PWD/results_plain scripts/results/reproduce_paper.sh all
PHOTONIC_ROUTER_NEGOTIATED_FIRST_ATTEMPT_FULL_BUDGET=1 RESULTS_ROOT=$PWD/results_b1 scripts/results/reproduce_paper.sh all
```
