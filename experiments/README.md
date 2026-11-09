# Experiments

Ideas that were tried against the paper table and did not become part of the
main code, and the records of the strategy switches that did. The main tree
carries only what the 27-cell table justified; everything else lives here,
reproducible, instead of in a scratch directory or a private notebook.

| experiment | status | one line |
| --- | --- | --- |
| [`port-runways/`](port-runways/) | not adopted (2026-11-09) | a static stub from every port onto the cell-centre lattice, so the search starts lattice-exact and the endpoint correction has nothing to move; costs +0.5 to +1.1 % length on Benes and four shallow arcs per net |
| [`crossing-hook-kernel-grid/`](crossing-hook-kernel-grid/) | not adopted (2026-11-06, follow-up 2026-11-09) | the live crossing hook reads the kernel's search grid instead of its own occupancy grid; eleven cells shorter, two cells fail; the dynamic expansion radius alone changes nothing |
| [`negotiation-first-attempt-full-budget/`](negotiation-first-attempt-full-budget/) | not adopted (2026-11-05) | the first search attempt runs at the retry budget; wall 0.99 in total, six lengths move |
| [`window-ladder/`](window-ladder/) | adopted (2026-11-05) | window 0 only, no full-grid fallback: every route ever found came from window 0; wall 0.91, three cells re-pinned |
| [`window-reachability-pruning/`](window-reachability-pruning/) | not adopted (2026-11-05) | a cell-level reachability test per search window to skip windows that cannot succeed; every failing window is cell-reachable |
| [`lto-build/`](lto-build/) | not adopted (2026-11-05) | fat link-time optimisation of the extension; no faster |
| [`negotiation-budget-schedule/`](negotiation-budget-schedule/) | adopted, exact (2026-11-09) | a fresh net's first attempt runs at the retry budget and the retry runs only when the first attempt stayed within the old first budget; same routes, the 2 M iterations of every budget-exhausted first attempt are no longer computed twice |

## What an entry holds

```
<name>/
  README.md     idea, what was built, measurement, result, decision, how to reproduce
  patches/      the code as `git format-patch` output against the base commit the README names
  scripts/      the measurement scripts, run from the repository root with the project venv
  tables/       compact records of the 27-cell runs (one JSON per table) and their rendering
  logs/         run logs when a table is not the right record
```

A table record (`tables/<variant>.json`) holds, per paper cell, the
benchmark, the configuration, the exit code, the wall seconds and the
`metrics.json` of the run that `scripts/results/reproduce_paper.sh` archived.
`experiments/table.py` renders one or more records against a reference
record or against the pinned table:

```bash
.venv/bin/python experiments/table.py --pins docs/paper_table_sources.json experiments/port-runways/tables/d3a62fe.json
.venv/bin/python experiments/table.py experiments/window-ladder/tables/reference.json experiments/window-ladder/tables/window0_only.json
```

## Rules

- An experiment that is not adopted leaves no switch in the main code. Its
  commits become patches here, the gate shows the tree without them exact,
  and the `CHANGELOG.md` entry names the folder.
- An adopted strategy switch keeps its code in the main tree and its record
  here: the tables that decided it and the re-pinned cells.
- Every number in a README comes from a file next to it, or names the
  commit and machine it was measured on. Wall times come from a shared
  server and compare only runs on the same machine.
- A patch applies to the base commit its README names. After later kernel
  changes it may need rebasing; the README says what the code did, so the
  idea survives the patch.
- An experiment that is reopened starts from its folder: the README says
  what was measured and why it closed, so the next attempt begins where the
  last one stopped.
