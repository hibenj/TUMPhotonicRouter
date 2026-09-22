"""Build the routing configuration for one run (Milestone 1, Slice 3 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`).

Precedence, lowest to highest: the dataclass defaults, the benchmark's
``STABLE_ROUTING_ENV`` block, the process environment. That is exactly what
the CLI did before through ``os.environ.setdefault`` (a variable already in
the environment beat the benchmark's block); the block is now applied through
the same overlay table as the environment, and the process environment is
never written. Command-line flags are separate: they remain keyword
arguments of ``run_routing_flow`` until Milestone 5 folds them in.
"""

from __future__ import annotations

import os
from collections.abc import Mapping

from photonic_router.config import RoutingConfig
from photonic_router.env_overlay import apply_env_overlay


def build_config(
    stable_env: Mapping[str, str] | None = None,
    environ: Mapping[str, str] | None = None,
) -> RoutingConfig:
    """Defaults, then the benchmark's stable block, then the environment.

    ``stable_env`` holds ``PHOTONIC_ROUTER_*`` names exactly as a benchmark
    module's ``STABLE_ROUTING_ENV`` declares them. ``environ`` defaults to
    ``os.environ``; pass a mapping in tests. A name present in both is taken
    from ``environ``. Values are parsed by the overlay's per-variable rules,
    so an invalid value raises here instead of at first use."""
    config: RoutingConfig = RoutingConfig()
    if stable_env:
        config = apply_env_overlay(config, stable_env)  # type: ignore[assignment]
    process_env = os.environ if environ is None else environ
    return apply_env_overlay(config, process_env)  # type: ignore[return-value]
