# Pattern pass in front of the kernel

Status: not pursued (2026-11-09), closed on the anatomy of the archived
table without building it. No patch.

## Idea

Research item 8 / speed pass B4: a generalised simple-route probe that tries
a small family of pattern routes (straight, L, Z and U with two 45-degree
bends) before the kernel's A*, so nets whose optimum is one of these shapes
never enter the search. 55 to 89 % of the nets already take the existing
straight simple route; the pass would extend that to the next shapes.

## Measurement

The ceiling of such a pass is the search time of the attempts it could
replace. Every attempt's expansion count and elapsed seconds are in the
`native_negotiated_search` trace lines of the archived table
(`scripts/expansion_buckets.py` over the `results_merged` table of
`../negotiation-budget-schedule/`, commit `1377ad6`, cda-server-4;
`tables/expansion_buckets_merged_1377ad6.md`). Over the 27 cells, 2,746 s
of search:

| attempts by expansion count | attempts | seconds | share |
| --- | --- | --- | --- |
| 0 (straight simple route) | 32,845 | 3.7 | 0.1 % |
| up to 10 k | 5,630 | 11.0 | 0.4 % |
| 10 k to 100 k | 2,198 | 84.5 | 3.1 % |
| 100 k to 1 M | 1,252 | 674.1 | 24.6 % |
| above 1 M | 320 | 1,647.7 | 60.0 % |
| failed attempts (any size) | 179 | 325 | 11.8 % |

On the heavy cells the small buckets are even smaller: ADEPT 64x64
lidar-pure spends 0.3 % of its 1,295 s on searches up to 100 k expansions
and 61 % on the 128 searches above 1 M; Benes 32x32 lidar-pure 0.7 % against
83 %.

## Result and decision

A pattern route replaces a search that would have found that pattern, and
such searches are short: everything up to 100 k expansions is 3.6 % of the
table's search time, and the pass cannot even take all of it (a pattern
route still has to be checked for legality and crossings, and it is only
exact where the kernel's optimum is that pattern). The time is in 1,572
searches above 100 k expansions, nets whose optimal route is not a two-bend
pattern. Not built. The 320 searches above 1 M expansions (60 % of the
search time) are the target of the heuristic and bounded-suboptimal
experiments (`../heuristic-tightness/`, `../weighted-astar/`).

## Reproduce

```bash
.venv/bin/python experiments/pattern-pass/scripts/expansion_buckets.py <results_root> multiportmmi_64x64/lidar-pure benes_32x32_flat/lidar-pure
```
