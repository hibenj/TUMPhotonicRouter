"""The Rust-backed routing session, split by pipeline phase.

`session` holds the session shell, one module per `run` phase holds that phase and
its stage-private helpers, and `api` holds the module-level entry points. The
phases answer to the Protocols in `stages`; they take the run's frozen inputs
(`settings.SessionSettings`) and its mutable record (`state.SessionState`), and
`timing` holds the pipeline timers they all use. The `translation.route_rust`
module is a compatibility shim re-exporting this package.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

from translation.routing.session import (  # noqa: E402
    DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    _RouteNetsRustSession,
    route_nets_rust,
)
from translation.routing.dispatch import NEGOTIATED_MAX_ROUNDS  # noqa: E402
from translation.routing.router_setup import (  # noqa: E402
    DENSE_OBSTACLE_CELL_CAP_MARGIN,
    dense_obstacle_cell_cap,
)
from translation.routing.api import (  # noqa: E402
    analyze_meander_insertion_for_requirements,
    insert_meanders_for_requirements,
    route_match_and_realize,
    static_fanout_anchors_um,
)

__all__ = [
    "DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING",
    "DENSE_OBSTACLE_CELL_CAP_MARGIN",
    "NEGOTIATED_MAX_ROUNDS",
    "_RouteNetsRustSession",
    "analyze_meander_insertion_for_requirements",
    "dense_obstacle_cell_cap",
    "insert_meanders_for_requirements",
    "route_match_and_realize",
    "route_nets_rust",
    "static_fanout_anchors_um",
]
