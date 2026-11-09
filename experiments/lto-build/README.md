# Fat link-time optimisation of the extension

Status: not adopted (2026-11-05). No patch: the change was
`lto = "fat"` and `codegen-units = 1` in a `[profile.release]` of the
workspace `Cargo.toml`, against the shipped profile, which sets neither
(Cargo's defaults: 16 codegen units, local thin LTO).

## Idea

Speed pass A1. The routing loop spends 88 % of its instructions in the Rust
extension; a fat LTO build with one codegen unit lets the optimiser inline
across the crate boundary of `router-core` and `router-py`.

## Measurement

Two builds on cda-server-4, 2026-11-05 10:32 to 10:41, back to back on the
same machine (load 3): the nine-cell gate twice per build, then Benes 16x16
lidar-pure and ADEPT 16x16 lidar-pure. Logs: `logs/benes16_adept16_thin_lto.log`,
`logs/benes16_adept16_fat_lto.log`.

| cell | routing loop s, shipped profile | routing loop s, fat LTO |
| --- | --- | --- |
| benes_16x16_flat lidar-pure | 124.4 | 128.7 |
| multiportmmi_16x16 lidar-pure | 24.7 | 25.7 |

Both builds reproduce the gate (`all reproduced cells match`, every metric
identical); over the nine gate cells the fat build's loop time is 1.02 of
the shipped build's (1 to 3 % slower on every cell, `t_loop` columns of the
two gate tables in the logs); the build takes 20 s instead of 8 s.

## Result and decision

No gain within the noise of a shared machine, slightly slower if anything.
The hot loops are memory-bound (the later profiling with cache simulation
showed the same), not call-bound. Not adopted; the profile stays as shipped.
