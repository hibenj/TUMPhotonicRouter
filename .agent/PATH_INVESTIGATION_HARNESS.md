# Path Investigation Harness

How a failing route is investigated in this repository. Written 2026-09-03 from
the mistakes of that day (multiportmmi_32x32 `n_286`, benes_32x32 nets 41/59/273);
each rule below names the mistake it prevents. The harness is mandatory for
every "net X does not route" investigation; the active ExecPlan records every
step BEFORE the next run (owner rule: "wir drehen uns im Kreis, weil die exec
plans nicht geupdated werden").

Tools: `scripts/path_investigation/grid_um.py` (grid <-> um from the flow's own
origin print), `scripts/path_investigation/gds_runs.py` (long straight runs and
parallel-neighbour pitch from the GDS), the env-gated diagnostics in
`src/astar.rs` / `src/py_router.rs` (listed in step 5), and a per-net watchdog.

## Step 0 -- Pin the state (before reading a single log line)

- Commit id of the code, and: is the deployed `python/photonic_router/_rust.abi3.so`
  the build of that commit? Never copy the .so while a flow process runs
  (`ps aux | grep routing_flow.py` first) -- a mapped .so overwritten under a
  running process dies with SIGBUS (exit 135). *Mistake: 12:23, killed the
  Milestone-0 run.*
- The exact configuration: the stdout must contain the line
  `Benchmark stable defaults applied: ...` with the expected flags (crossing
  mode, fanout access, iteration cap, congestion weights) and every env var
  you set. A bare `routing_flow.py <bench>` runs whatever the benchmark file's
  `STABLE_ROUTING_FLAGS` say -- or the flow defaults (`--crossing-mode window`)
  if the file has none. *Mistake: the first benes_32x32 run of the day was
  window mode; two hours of net-41 analysis were about the wrong mode.*
- The grid mapping from the tool, never by hand:
  `PHOTONIC_ROUTER_TRACE_GRID=1 python routing_flow.py <bench> --debug-stop-after-route 1`
  prints `grid: origin=(ox, oy) um size=s`; um = origin + (cell + 0.5) * size
  (`grid_um.py`). *Mistake: hand-converted coordinates 60 um off, a whole
  argument built on the neighbouring diagonal.*

## Step 1 -- Localize: which net, which attempt

- Net: index, rust net id, name, source/target in cells AND um, both port
  specs (instance/port), whether the target port is one of a dense pair
  (Benes switch inputs are one row apart).
  `PHOTONIC_ROUTER_NATIVE_PROGRESS=1` prints `native_route_start index=.. net_id=.. source=.. target=..`.
- Attempt: every diagnostic line carries a per-search `seq`. Before reading
  ANY failure line, establish which attempt it belongs to: full-map
  lidar-pure search (partner set = all committed routes) or a single-partner
  repair search (`partners=[N]`, `unexpected_owner` for every other net).
  `PHOTONIC_ROUTER_TRACE_CROSSING_NET=<id>` prints `collision-crossing start
  net=.. partners=[..] lookup_partner_ids=[..] committed_center_routes=[..]`.
  *Mistake: the "wall at y=1521" was a single-partner repair search; the
  full search had succeeded.*
- Diagnostic lines are printed DURING a search, i.e. BEFORE that search's
  `search-failure seq=N` header: they belong to the FOLLOWING header.
  *Mistake: blocker lines attributed to the wrong search.*
- Failure kind: `iteration_cap` is not a verdict -- raise the cap (mm32 and
  benes_32x32 stable blocks carry `--max-iterations 20000000`) until the
  search reports `open_set_exhausted`; only then is "no route in this window"
  a fact. *Mistake: net 59 looked like a search-cost problem at 5M.*

## Step 2 -- Freeze the geometry: GDS before the failing net

- `--debug-stop-after-route N-1` (N includes the failing net!) and snapshot
  the GDS with a name that says what it is (`build/routed_<bench>_<tag>_net<N-1>.gds`).
  Note attempts/failures/repairs of that run: if repairs > 0 the state already
  differs from the first-pass state. *Mistake: stop-287 delivered a GDS
  including the failing net.*
- The investigation, the GDS and the path trace must all be from the SAME
  build and configuration. Re-generate after every code or flag change --
  never argue from yesterday's GDS or an older log. *Mistake: argued on an
  old svg/log while the GDS was current.*

## Step 3 -- Read the geometry: what would a legal path need?

With the crossing rules (one number `half_size` from the GDS crossing; two
predicates: only straight cells inside the +-half_size window on both nets,
and reservation windows disjoint):

- Corridor between source and target: list the committed runs the route must
  cross (`gds_runs.py --dir v|h|d --x .. --y ..`). Parallel runs closer than
  5 cells (10 um at grid 2 um) cannot both be crossed by one straight; a run
  shorter than `half_size` on either side of a crossing point cannot be
  crossed there; runs that must be crossed non-perpendicularly cannot be
  crossed at all.
- Target approach: the port stub (opened rows), the neighbouring port's stub
  (static, NOT opened for this net) and the neighbour's committed route
  (dynamic core one row away); whether a free pocket exists in front of the
  stub and from which side it is reachable.
- Result of this step, written down: the expected path (waypoints, crossing
  list with "feasible / not feasible / avoidable" per crossing) -- this is
  the acceptance criterion the owner signs off (as for `n_286`: "cross n_280
  and the parallels below on their diagonals with one straight line").
  *Mistake: chasing the search's failing path instead of first writing the
  path the geometry allows.*

## Step 4 -- Search evidence, filtered by seq

Env-gated diagnostics (all permanent):
- `PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG=1`: `search-failure seq=.. kind=..
  explored_bbox=..`, `search-failure-bestpath crossings=..` (the path with the
  most accepted crossings), `search-failure-ring` (25 cells around the target:
  static/opened + landing counters gen/acc/foot/hook/closed/pruned/resv),
  `search-failure-blocker` / `-profile` (capped per ring slot, 3 lines each --
  a global cap hid the relevant side for half a day).
- `PHOTONIC_ROUTER_PROBE_CELLS="x,y;.."`: obstacle-map view
  (static/opened/owners/core) AND the dense grid's `dense_blocked` -- the
  search's own view. Ask "is this cell blocked for the search?" with
  `dense_blocked`, never by reasoning about halos. *Mistake: a halo
  hypothesis refuted by one probe.*
- `PHOTONIC_ROUTER_MOVE_DIAG="x,y"`: counter deltas for every hook rejection
  landing there. Filter by `seq=` -- lines from earlier nets share the file.
  *Mistake: read seq=3 lines as the net-59 search.*
- `PHOTONIC_ROUTER_TRACE_CROSSING_{LEVEL1,CANDIDATES,PENDING}=1` with
  `TRACE_CROSSING_NET`: per-intersection accept/reject reasons (candidate
  trace is capped -- check the cap before concluding "no reject of kind X").
- `PHOTONIC_ROUTER_CHAIN_DIAG=1`: eager-completion-chain breaks.
- A hypothesis is only adopted with a prediction and the diag that tests it
  in one targeted run (stop-after-route N, kill after the relevant window).
  Refutations are written down, not deleted. *Mistakes: the 45-degree gap,
  the arm-crossing hole, the halo, the 4-cell entry -- all retracted by one
  more probe or one more function read.*

## Step 5 -- Post-search evidence: was a route found and thrown away?

`collision-crossing result net=..` + `collision-crossing validation net=..
satisfies=.. no_unresolved_grid_crossings=.. partner_constraints=..
realized_violations=..`. Every post-search reject must (a) be traced with its
sub-condition and (b) exist as a search-time rule -- otherwise the search
keeps producing routes the validator refuses. *Mistake twice: zero-event
routes (2026-09-02) and overlapping own windows (2026-09-03) discarded
silently.*

## Step 6 -- Fix or tune, then prove

- Kernel fix: unit test first (pin the rule in a small grid), then cargo
  suite, pytest baseline, five-benchmark ladder, the affected 32x32 full run.
  A rule number gets one name and one meaning (`required_margin` mixed three).
- Tuning experiment (owner-directed, e.g. E1 in the benes plan): one knob at
  a time, penalised AREA unchanged, WEIGHT raised; success rule written before
  the runs (geometry criterion + attempts/failures/repairs + verifications).
- Runs: `--debug-stop-after-route` granularity, and a per-net watchdog of
  3 minutes -- a net that has not advanced in 3 minutes is the finding; a 3 h
  timeout is not. *Mistake: 3 h timeouts and repair cascades that grind for
  half an hour with zero information.*

## Step 7 -- Record

Before the next run: plan entry with the seq, the numbers, the GDS names and
the verdict (confirmed / refuted). After the fix: retrospective line naming
which of the rules above would have caught it earlier.
