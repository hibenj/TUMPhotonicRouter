"""The Rust-backed routing session, split by pipeline phase.

`session` holds the session shell, one module per `run` phase holds that phase and
its stage-private helpers, and `api` holds the module-level entry points. The
phases answer to the Protocols in `stages`; they take the run's frozen inputs
(`settings.SessionSettings`) and its mutable record (`state.SessionState`), and
`timing` holds the pipeline timers they all use. The `photonic_router.routing.route_rust`
module is a compatibility shim re-exporting this package.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

from photonic_router.routing.api import (  # noqa: E402
    analyze_meander_insertion_for_requirements,
    route_match_and_realize,
    static_fanout_anchors_um,
)
from photonic_router.routing.dispatch import NEGOTIATED_MAX_ROUNDS  # noqa: E402
from photonic_router.routing.router_setup import (  # noqa: E402
    DENSE_OBSTACLE_CELL_CAP_MARGIN,
    dense_obstacle_cell_cap,
)
from photonic_router.routing.session import (  # noqa: E402
    _RouteNetsRustSession,
    route_nets_rust,
)

__all__ = [
    "DENSE_OBSTACLE_CELL_CAP_MARGIN",
    "NEGOTIATED_MAX_ROUNDS",
    "_RouteNetsRustSession",
    "analyze_meander_insertion_for_requirements",
    "dense_obstacle_cell_cap",
    "route_match_and_realize",
    "route_nets_rust",
    "static_fanout_anchors_um",
]
