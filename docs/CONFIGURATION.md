# Configuration reference

<!-- GENERATED FILE. Do not edit by hand. Regenerate with
     `.venv/bin/python scripts/generate_config_reference.py`
     and verify with `--check` (also asserted by
     `tests/test_config_reference.py`). -->

Every knob the router takes, generated from the dataclasses themselves by
`scripts/generate_config_reference.py`. Two trees configure a run and nothing
else does:

* `RoutingConfig` (`python/photonic_router/config.py`) -- the router's own
  settings, one typed field per former `PHOTONIC_ROUTER_*` environment
  variable (Milestone 1). Its `router` subtree mirrors Rust `RouterConfig`
  (`src/config.rs`) field for field and default for default, and is what
  `RouterConfig.to_rust` hands to `rust_backend.PyPhotonicRouter`.
* `FlowOptions` (`python/photonic_router/flow_options.py`) -- the flow-level
  choices: which benchmark, which stages run, which artifacts are written.
  It is the second argument of `photonic_router.flow.route_benchmark`.

Precedence, lowest to highest (`python/photonic_router/config_loading.py`,
`python/photonic_router/cli.py::main`): the dataclass defaults below, then the
benchmark module's `STABLE_ROUTING_ENV` block, then the process environment,
then the command line. The environment column names the overlay variable that
reaches a field (`python/photonic_router/env_overlay.py`'s `ENV_OVERLAY`); the
command-line column names the flag `photonic_router.cli::_flow_options` fills a
field from. A field with no variable and no flag is set programmatically only.
A field of the `router` subtree with an empty description is documented by its
Rust twin's doc comment in `src/config.rs`, which these dataclasses mirror.
See `docs/ARCHITECTURE.md` for how the two trees reach the stages.

Totals: 88 `RoutingConfig` fields (88 with an overlay variable), 48 `FlowOptions` fields (40 with a command-line flag), 136 in total.

## `RoutingConfig` -- the router's settings

### `RoutingConfig` -- `RoutingConfig`

Milestone 1, Slice 2: the top-level Python-side configuration tree. Every `PHOTONIC_ROUTER_*` variable read by `translation/` and `routing_flow*.py` (outside `env_overlay.py` itself) is a typed field somewhere under this tree, threaded explicitly from `run_routing_flow` down to `_RouteNetsRustSession` (whose `.router` field is `RouterConfig`, the Rust-boundary configuration Milestone 1 Slice 1 introduced).

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `write_gds_on_photonic_verification_failure` | `bool` | `False` | `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE` | `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE` (`routing_flow_verification.py`): `.strip().lower() in {"1", "true", "yes", "on"}`, default False. |

### `router` -- `RouterConfig`

Mirrors Rust `RouterConfig`: the top-level configuration tree passed explicitly into `rust_backend.PyPhotonicRouter`, replacing every `PHOTONIC_ROUTER_*` variable the kernel used to read directly.

No fields of its own.

### `router.negotiation` -- `NegotiationConfig`

Mirrors Rust `NegotiationConfig`.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `budget_first` | `int` | `2000000` | `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST` | -- |
| `budget_first_retry` | `int` | `10000000` | `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST_RETRY` | -- |
| `budget_retry` | `int` | `30000000` | `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_RETRY` | -- |
| `braid_escalation` | `bool` | `False` | `PHOTONIC_ROUTER_NEGOTIATED_BRAID_ESCALATION` | -- |
| `crossing_free_unplanned` | `bool` | `True` | `PHOTONIC_ROUTER_NEGOTIATED_CROSSING_FREE_UNPLANNED` | -- |
| `disable_braid_repair` | `bool` | `False` | `PHOTONIC_ROUTER_DISABLE_BRAID_REPAIR` | -- |
| `pending_straight_ripup_threshold` | `int` | `100` | `PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD` | -- |
| `enable_orthogonal_repair_fallback` | `bool` | `False` | `PHOTONIC_ROUTER_ENABLE_ORTHOGONAL_REPAIR_FALLBACK` | -- |

### `router.search` -- `SearchOverrides`

Mirrors Rust `SearchOverrides`. `None` means "no override" for every field, not any particular value.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `astar_timeout_ms` | `int \| None` | `None` | `PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS`, `PHOTONIC_ROUTER_ASTAR_TIMEOUT_S` | -- |
| `max_dense_states` | `int \| None` | `None` | `PHOTONIC_ROUTER_MAX_DENSE_STATES` | -- |
| `long_straight_congestion_weight` | `float \| None` | `None` | `PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT` | -- |
| `engine` | `str` | `'astar'` | `PHOTONIC_ROUTER_SEARCH_ENGINE` | `PHOTONIC_ROUTER_SEARCH_ENGINE`: which Rust `NetSearch` engine routes a single net -- `"astar"` (default) or `"grid-dijkstra"`. Unlike the three above this one is never "unset"; the Rust binding rejects any other value. |

### `router.crossing` -- `CrossingEngineConfig`

Mirrors Rust `CrossingEngineConfig`.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `enable_guided_collision_crossing` | `bool` | `False` | `PHOTONIC_ROUTER_ENABLE_GUIDED_COLLISION_CROSSING` | -- |
| `disable_guided_collision_crossing` | `bool` | `False` | `PHOTONIC_ROUTER_DISABLE_GUIDED_COLLISION_CROSSING` | -- |
| `disable_rust_crossing_validation` | `bool` | `False` | `PHOTONIC_ROUTER_DISABLE_RUST_CROSSING_VALIDATION` | -- |

### `router.diagnostics` -- `KernelDiagnostics`

Mirrors Rust `KernelDiagnostics`. Every field is off/unset by default, except `trace_crossing_candidate_max` (120, matching the kernel's own fallback).

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `native_progress` | `bool` | `False` | `PHOTONIC_ROUTER_NATIVE_PROGRESS` | -- |
| `native_repair_diag` | `bool` | `False` | `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG` | -- |
| `search_failure_diag` | `bool` | `False` | `PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG` | -- |
| `search_failure_map` | `tuple[int, ...] \| None` | `None` | `PHOTONIC_ROUTER_SEARCH_FAILURE_MAP` | -- |
| `chain_diag` | `bool` | `False` | `PHOTONIC_ROUTER_CHAIN_DIAG` | -- |
| `hot_loop_timing` | `bool` | `False` | `PHOTONIC_ROUTER_HOT_LOOP_TIMING` | -- |
| `move_diag` | `bool` | `False` | `PHOTONIC_ROUTER_MOVE_DIAG` | -- |
| `move_diag_cell` | `tuple[int, int] \| None` | `None` | `PHOTONIC_ROUTER_MOVE_DIAG` | -- |
| `pop_diag_below_y` | `int \| None` | `None` | `PHOTONIC_ROUTER_POP_DIAG_BELOW_Y` | -- |
| `probe_cells` | `tuple[tuple[int, int], ...]` | `()` | `PHOTONIC_ROUTER_PROBE_CELLS` | -- |
| `trace_crossing` | `bool` | `False` | `PHOTONIC_ROUTER_TRACE_CROSSING` | -- |
| `trace_crossing_net` | `int \| None` | `None` | `PHOTONIC_ROUTER_TRACE_CROSSING_NET` | -- |
| `trace_crossing_candidates` | `bool` | `False` | `PHOTONIC_ROUTER_TRACE_CROSSING_CANDIDATES` | -- |
| `trace_crossing_candidate_max` | `int` | `120` | `PHOTONIC_ROUTER_TRACE_CROSSING_CANDIDATE_MAX` | -- |
| `trace_crossing_level1` | `bool` | `False` | `PHOTONIC_ROUTER_TRACE_CROSSING_LEVEL1` | -- |
| `trace_crossing_pending` | `bool` | `False` | `PHOTONIC_ROUTER_TRACE_CROSSING_PENDING` | -- |
| `trace_crossing_pending_threshold` | `int \| None` | `None` | `PHOTONIC_ROUTER_TRACE_CROSSING_PENDING_THRESHOLD` | -- |
| `trace_crossing_perp_reject_threshold` | `int \| None` | `None` | `PHOTONIC_ROUTER_TRACE_CROSSING_PERP_REJECT_THRESHOLD` | -- |
| `trace_partner_net` | `int \| None` | `None` | `PHOTONIC_ROUTER_TRACE_PARTNER_NET` | -- |
| `trace_plain_route_net` | `int \| None` | `None` | `PHOTONIC_ROUTER_TRACE_PLAIN_ROUTE_NET` | -- |
| `trace_endpoint_bump_nets` | `None \| tuple[str, ...] \| str` | `None` | `PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS` | `None` (no trace), `config.ALL_NETS` ("*", every net), or a tuple of net-id strings -- see `NetNameTrace` above. |
| `trace_endpoint_correction_net` | `int \| None` | `None` | `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NET` | -- |
| `crossing_mismatch_dump` | `bool` | `False` | `PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP` | -- |
| `crossing_mismatch_dump_net` | `int \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP` | -- |
| `crossing_mismatch_fatal` | `bool` | `False` | `PHOTONIC_ROUTER_CROSSING_MISMATCH_FATAL` | -- |
| `analysis_crossing_partner_counters` | `bool` | `False` | `PHOTONIC_ROUTER_ANALYSIS_CROSSING_PARTNER_COUNTERS` | -- |

### `crossing_plan` -- `CrossingPlanConfig`

Mirrors the env reads of `translation/route_rust_crossing_plan.py`. Every field is `None` when unset, in which case the site keeps its own literal default (named below).

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `collision_crossing_search_loss_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM` | `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM`: float, finite and non-negative. `None` = unset (site default `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM` = 200.0). |
| `planned_crossing_search_loss_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_PLANNED_CROSSING_SEARCH_LOSS_UM` | `PHOTONIC_ROUTER_PLANNED_CROSSING_SEARCH_LOSS_UM`: float, finite and non-negative. `None` = unset (site default `DEFAULT_PLANNED_CROSSING_SEARCH_LOSS_UM` = 0.0). |
| `planned_crossing_budget` | `bool \| None` | `None` | `PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET` | `PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET`: `"0"` -> False, `"1"` -> True, anything else raises. `None` = unset (site default `DEFAULT_SINGLE_DISCOUNTED_CROSSING_PER_PAIR` = False). |

### `crossing_grid` -- `CrossingGridConfig`

Mirrors the env reads of `translation/preplaced_crossing_grids.py`'s `crossing_grid_geometry_from_config`/`_column_grid_stage` and `translation/crossing_structures.py`'s `select_layer_structure`. Every float/string field is `None` when unset, in which case the site's own (`CrossingGridGeometry` or literal) default applies.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `band_margin_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_BAND_MARGIN_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_BAND_MARGIN_UM` (default 50.0). |
| `bend_radius_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_BEND_RADIUS_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_BEND_RADIUS_UM` (default 5.0). |
| `column_lead_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_LEAD_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_LEAD_UM` (default 14.0). |
| `column_pitch_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_PITCH_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_PITCH_UM` (default 16.0). |
| `corner_margin_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_CORNER_MARGIN_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_CORNER_MARGIN_UM` (default 14.0). |
| `entry_straight_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_ENTRY_STRAIGHT_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_ENTRY_STRAIGHT_UM` (default 4.0). |
| `fan_column_pitch_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_FAN_COLUMN_PITCH_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_FAN_COLUMN_PITCH_UM` (default 4.0). |
| `lane_pitch_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM` (default 14.0). |
| `port_pair_spread_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_PORT_PAIR_SPREAD_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_PORT_PAIR_SPREAD_UM` (default 2.0). |
| `slot_spread_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_SLOT_SPREAD_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_SLOT_SPREAD_UM` (default 11.0). |
| `stub_stagger_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_STUB_STAGGER_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_STUB_STAGGER_UM` (default 6.0). |
| `tile_min_spacing_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_TILE_MIN_SPACING_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_TILE_MIN_SPACING_UM` (default 14.0). |
| `unrouted_sibling_clearance_um` | `float \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_UNROUTED_SIBLING_CLEARANCE_UM` | `PHOTONIC_ROUTER_CROSSING_GRID_UNROUTED_SIBLING_CLEARANCE_UM` (default 0.0). |
| `fan_mode` | `str \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_FAN_MODE` | `PHOTONIC_ROUTER_CROSSING_GRID_FAN_MODE`: string (site default `base.fan_mode`, "tiles"); a blank value is also "unset". |
| `tile_placement` | `str \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_TILE_PLACEMENT` | `PHOTONIC_ROUTER_CROSSING_GRID_TILE_PLACEMENT`: string (site default "auto"); a blank value is also "unset". |
| `corners` | `str \| None` | `None` | `PHOTONIC_ROUTER_CROSSING_GRID_CORNERS` | `PHOTONIC_ROUTER_CROSSING_GRID_CORNERS`: string (site default "always"); a blank value is also "unset". |
| `router_layers` | `frozenset[int]` | `frozenset()` | `PHOTONIC_ROUTER_CROSSING_GRID_ROUTER_LAYERS` | `PHOTONIC_ROUTER_CROSSING_GRID_ROUTER_LAYERS`: comma-separated ints (non-digit tokens dropped); default empty. |
| `trace_column_grid` | `bool` | `False` | `PHOTONIC_ROUTER_TRACE_COLUMN_GRID` | `PHOTONIC_ROUTER_TRACE_COLUMN_GRID`: truthy string. |

### `fanout` -- `FanoutAccessConfig`

Mirrors the fan-out-related env reads of `translation/route_rust.py`.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `dense_fanout_instances` | `frozenset[str] \| None` | `None` | `PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES` | `PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES`: comma-separated instance names. Unset (`None`) means no override. Also SELF-SET: the pre-placed crossing-grid stage computes this set and `run_routing_flow` folds it into the routing-stage config (see `translation/preplaced_crossing_grids.py`'s `_derive_crossing_tiles`). |
| `dense_fanout_min_ports` | `int \| None` | `None` | `PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS` | `PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS`: int >= 2, else raises. `None` = unset (site default 3). |
| `fanout_access_mode` | `str \| None` | `None` | `PHOTONIC_ROUTER_FANOUT_ACCESS_MODE` | `PHOTONIC_ROUTER_FANOUT_ACCESS_MODE`: today this OVERRIDES the constructor argument (`os.environ.get(NAME, default_or_ctor_arg)`, then alias-normalized). `None` = unset -> the constructor argument/default is used. |
| `fanout_lane_spacing_cells` | `int \| None` | `None` | `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS` | `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS`: non-negative int, else raises. `None` = unset (site defaults: 11 at `_build_static_fanout_anchors`, 3 at the three other sites). |
| `fanout_protected_lane_spacing_cells` | `int \| None` | `None` | `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS` | `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS`: non-negative int, else raises. `None` = unset -> falls back to `fanout_lane_spacing_cells`, then 3. |
| `target_protected_lane_spacing_cells` | `int \| None` | `None` | `PHOTONIC_ROUTER_TARGET_PROTECTED_LANE_SPACING_CELLS` | `PHOTONIC_ROUTER_TARGET_PROTECTED_LANE_SPACING_CELLS`: non-negative int, else raises. `None` = unset -> outermost of that fallback chain. |
| `fanout_stub_bend_degrees` | `str \| None` | `None` | `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES` | `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES`: alias table (see `_fanout_stub_bend_steps`), site default `"90"`. `None` = unset. |
| `fanout_stub_forward_cells` | `int \| None` | `None` | `PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS` | `PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS`: non-negative int, else raises. `None` = unset (site computes `max(3, bend_radius_cells + 3)`). |
| `fanout_stub_x_offset_cells` | `int \| None` | `None` | `PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS` | `PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS`: non-negative int, else raises. `None` = unset (site default 1). |
| `stub_port_lane_half_width_cells` | `int \| None` | `None` | `PHOTONIC_ROUTER_STUB_PORT_LANE_HALF_WIDTH_CELLS` | `PHOTONIC_ROUTER_STUB_PORT_LANE_HALF_WIDTH_CELLS`: non-negative int, else raises. `None` = unset (site default 0). |
| `stub_port_lane_length_cells` | `int \| None` | `None` | `PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH_CELLS` | `PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH_CELLS`: non-negative int, else raises. `None` = unset (site default 0). |

### `engine` -- `EngineSelection`

Mirrors the repair-engine selection env reads of `translation/route_rust.py`'s `negotiated_repair_engine_enabled`.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `negotiated_repair` | `bool` | `True` | `PHOTONIC_ROUTER_NEGOTIATED_REPAIR` | `PHOTONIC_ROUTER_NEGOTIATED_REPAIR`: `!= "0"`, default True. |
| `legacy_repair_chain` | `bool` | `False` | `PHOTONIC_ROUTER_LEGACY_REPAIR_CHAIN` | `PHOTONIC_ROUTER_LEGACY_REPAIR_CHAIN`: `== "1"`, default False. When True it forces the legacy chain regardless of `negotiated_repair`. |

### `search` -- `SearchTuning`

Mirrors the search-tuning env reads of `translation/route_rust.py`.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `min_bend_weight` | `float` | `12.0` | `PHOTONIC_ROUTER_MIN_BEND_WEIGHT` | `PHOTONIC_ROUTER_MIN_BEND_WEIGHT`: float, default 12.0. |
| `min_heuristic_weight` | `float` | `1.0` | `PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT` | `PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT`: float, default 1.0. |
| `heap_tie_breaker` | `str \| None` | `None` | `PHOTONIC_ROUTER_HEAP_TIE_BREAKER` | `PHOTONIC_ROUTER_HEAP_TIE_BREAKER`: only exactly `"smaller_g"` or `"larger_g"` override; any other value (including unset) means `None`, and the site computes its own default. |
| `long_straight_exempt_dense_fanout` | `bool \| None` | `None` | `PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT` | `PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT`: `== "1"`. `None` = unset -- SELF-SET by `routing_flow.py` to True when pre-placed crossing grids are enabled and this field is still `None`; the site tests `is True`. |

### `diagnostics` -- `FlowDiagnostics`

Mirrors the Python-side diagnostic env reads of `translation/route_rust.py` and `translation/route_rust_endpoint_correction.py`.

| field | type | default | environment variable | description |
| --- | --- | --- | --- | --- |
| `debug_execution_limit` | `int \| None` | `None` | `PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT` | `PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT`: int >= 1, else raises. `None` = unset. |
| `debug_route_first_instance` | `str` | `''` | `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_INSTANCE` | `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_INSTANCE`: string, default `""`. |
| `debug_route_first_nets` | `tuple[int, ...]` | `()` | `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS` | `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS`: comma-separated ints, list order kept (non-digit tokens dropped); default empty. |
| `trace_endpoint_correction_nets` | `frozenset[str]` | `frozenset()` | `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS` | `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS`: comma-separated set of net names; default empty. |
| `trace_fanout_stubs` | `bool` | `False` | `PHOTONIC_ROUTER_TRACE_FANOUT_STUBS` | `PHOTONIC_ROUTER_TRACE_FANOUT_STUBS`: non-empty string (after `.strip()`) is truthy. |
| `trace_grid` | `bool` | `False` | `PHOTONIC_ROUTER_TRACE_GRID` | `PHOTONIC_ROUTER_TRACE_GRID`: truthy string. |
| `trace_runway_instance` | `str \| None` | `None` | `PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE` | `PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE`: string; truthy then re-read. `None` = unset (a blank value is also "unset"). |
| `trace_terminal_bump_distance_checks` | `frozenset[str]` | `frozenset()` | `PHOTONIC_ROUTER_TRACE_TERMINAL_BUMP_DISTANCE_CHECKS` | `PHOTONIC_ROUTER_TRACE_TERMINAL_BUMP_DISTANCE_CHECKS`: comma set of tokens, `"*"` matches every net; default empty. |

## `FlowOptions` -- the flow-level choices

### `FlowOptions` -- `FlowOptions`

Every flow-level choice, grouped by consuming stage.

No fields of its own.

### `loading` -- `LoadingOptions`

Stage 1, `load_benchmark`: which benchmark module the flow routes.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `benchmark_name` | `str` | `'benes_16x16'` | `<benchmark>` (positional) | -- |

### `layout` -- `LayoutOptions`

Waveguide geometry the layout and every routing stage must agree on. Read by the optical stage's primitive library, and `bend_radius_um` also by the pre-placed crossing grid stage's static-fan-out probe, which has to see the routing run's own geometry.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `allow_45_degree_turns` | `bool` | `True` | `--allow-45-degree-turns` | -- |
| `bend_radius_um` | `float` | `5.0` | `--bend-radius-um` | -- |

### `obstacles` -- `ObstacleOptions`

`build_static_obstacle_config`: the static optical obstacle map. `include_heater_obstacles` decides whether the heater/metal layers are obstacles at all, so the optical stage, the verification stage and the pre-placed grid probe all read it from here.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `static_obstacle_config` | `StaticObstacleMapConfig \| None` | `None` | `--grid-size-um`, `--obstacle-mode`, `--obstacle-clearance-um`, `--heater-clearance-um`, `--chip-add-x-um`, `--chip-add-y-um`, `--clear-port-open-cells-from-static` | -- |
| `grid_size_um` | `float` | `2.0` | `--grid-size-um` | -- |
| `waveguide_clearance_um` | `float \| None` | `None` | `--obstacle-clearance-um` | -- |
| `heater_clearance_um` | `float \| None` | `None` | `--heater-clearance-um` | -- |
| `obstacle_clearance_um` | `float \| None` | `None` | -- | -- |
| `chip_add_x_um` | `float` | `0.0` | `--chip-add-x-um` | -- |
| `chip_add_y_um` | `float` | `0.0` | `--chip-add-y-um` | -- |
| `include_heater_obstacles` | `bool` | `False` | `--include-heater-obstacles` | -- |

### `preplaced_grids` -- `PreplacedCrossingGridOptions`

Contribution 2: the pre-placed crossing grid stage (stage 2b).

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `preplaced_crossing_grids` | `bool` | `False` | `--preplaced-crossing-grids` | -- |

### `optical` -- `OpticalStageOptions`

`build_optical_routing_stage_config`: the optical routing stage.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `enable_crossings` | `bool` | `False` | `--crossings` | -- |
| `crossing_mode` | `str` | `'window'` | `--crossing-mode` | -- |
| `crossing_half_size_cells` | `int` | `0` | -- | -- |
| `min_straight_cells_per_crossing` | `int` | `2` | `--min-straight-cells-per-crossing` | -- |
| `foreign_port_keepout_cells` | `int` | `6` | `--foreign-port-keepout-cells` | -- |
| `fanout_access_mode` | `str \| None` | `'legacy-runway'` | `--fanout-access-mode` | -- |
| `proactive_congestion_weight` | `float` | `0.0` | `--proactive-congestion-weight` | -- |
| `proactive_congestion_radius_cells` | `int` | `0` | `--proactive-congestion-radius-cells` | -- |
| `enable_jps4` | `bool` | `False` | `--enable-jps4` | -- |
| `use_indexed_heap` | `bool` | `False` | `--use-indexed-heap` | -- |
| `enable_simple_routes` | `bool` | `True` | `--enable-simple-routes` | -- |
| `primitive_ordering` | `str` | `'library'` | `--primitive-ordering` | -- |
| `heuristic_mode` | `str` | `'heading_aware'` | `--heuristic-mode` | -- |
| `net_order` | `str \| None` | `None` | `--net-order` | -- |
| `heap_tie_breaker` | `str` | `'smaller_g'` | `--heap-tie-breaker` | -- |
| `max_iterations` | `int` | `500000` | `--max-iterations` | -- |
| `routing_window_scale` | `float \| None` | `None` | `--routing-window-scale` | -- |
| `ripup_reroute_config` | `RipupRerouteConfig \| None` | `None` | `--ripup-reroute`, `--ripup-max-rounds`, `--ripup-max-victims`, `--ripup-history-weight`, `--ripup-history-increment` | -- |

### `verification` -- `VerificationOptions`

`verify_and_attach_photonic_reports`: the post-route verification stage. Deliberately empty: the stage always runs, and its three inputs come from elsewhere - `obstacles.include_heater_obstacles`, `debug.debug_stop_after_route_index` and `RoutingConfig.write_gds_on_photonic_verification_failure`. The group is named so the boundary is visible in the option tree.

No fields of its own.

### `path_length` -- `PathLengthMatchingOptions`

`attach_and_report_path_length_matching` and the optical stage's path-length requirements.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `enable_path_length_matching` | `bool` | `False` | `--path-length-matching` | -- |
| `path_length_match_outputs` | `bool` | `False` | `--path-length-match-outputs` | -- |
| `path_length_meander_height_um` | `float` | `80.0` | `--path-length-meander-height-um` | -- |

### `electrical` -- `ElectricalOptions`

`run_electrical_routing_step`: the heater-metal routing stage.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `enable_electrical_routing` | `bool` | `False` | `--electrical-routing` | -- |
| `electrical_config` | `ElectricalRoutingConfig \| None` | `None` | `--electrical-pad-side`, `--electrical-grid-pitch-um`, `--electrical-obstacle-clearance-um`, `--electrical-wire-width-um`, `--electrical-bus-width-um`, `--electrical-terminal-contact-width-um`, `--electrical-pad-pitch-um` | -- |

### `debug` -- `DebugArtifactOptions`

Debug artifacts and verbosity: SVGs, timings, per-net prints. `show_debug_svgs` and `show_static_obstacles_svg` are the legacy aliases `resolve_legacy_display_options` folds into `debug_svgs`.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `debug_svgs` | `DebugSvgSelector` | `False` | `--debug-svgs` | -- |
| `show_debug_svgs` | `DebugSvgSelector \| None` | `None` | -- | -- |
| `show_static_obstacles_svg` | `bool \| None` | `None` | -- | -- |
| `debug_timing` | `bool` | `False` | `--debug-timing` | -- |
| `debug_stop_after_route_index` | `int \| None` | `None` | `--debug-stop-after-route` | -- |
| `debug_meanders` | `bool` | `False` | `--debug-meanders` | -- |
| `verbose_routes` | `bool` | `False` | `--verbose-routes` | -- |
| `collect_attempt_diagnostics` | `bool` | `False` | `--attempt-diagnostics` | -- |

### `output` -- `OutputOptions`

`write_or_show_routed_layout`: where the routed layout goes. `show_routed` and `show_unrouted` are the legacy aliases `resolve_legacy_display_options` folds into `show_klayout`.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `show_klayout` | `bool` | `False` | `--show-klayout` | -- |
| `show_routed` | `bool \| None` | `None` | -- | -- |
| `show_unrouted` | `bool \| None` | `None` | -- | -- |

### `stats` -- `StatsOptions`

The optional legacy stats collector filled in place by the stages.

| field | type | default | command line | description |
| --- | --- | --- | --- | --- |
| `collect_route_stats` | `bool` | `False` | -- | -- |
| `stats` | `RoutingFlowStats \| None` | `None` | -- | -- |
