"""Benchmark metadata loading and component-derived delay resolution."""

from __future__ import annotations

import importlib
from typing import Any, Callable, Protocol, cast

class SchematicLike(Protocol):
    netlist: Any


def _coerce_nonnegative_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return max(0.0, float(cast(Any, value)))
    except (TypeError, ValueError):
        return None


def is_auto_internal_delay_value(value: Any) -> bool:
    if value is None:
        return True
    if isinstance(value, str):
        return value.strip().lower() in {
            "auto",
            "auto-detect",
            "auto_detect",
            "auto detect",
            "infer",
            "infer_from_component",
            "from_component",
            "from-component",
        }
    return False


def coerce_instance_component_name(instance: Any) -> str | None:
    component_obj = getattr(instance, "component", None)
    if component_obj is None:
        return None
    if isinstance(component_obj, str):
        return component_obj
    name = getattr(component_obj, "name", None)
    if isinstance(name, str) and name:
        return name
    return None


def component_internal_delay_um(component_name: str) -> float | None:
    try:
        from gdsfactory import get_component

        component = get_component(component_name)
    except Exception:
        component = None

    if component is None:
        try:
            from gdsfactory.generic_tech import get_generic_pdk

            pdk = get_generic_pdk()
            pdk.activate()
            pdk_cell = getattr(pdk, "cell", None)
            component = pdk_cell(component_name) if callable(pdk_cell) else None
        except Exception:
            component = None

    if component is None:
        return None

    info = getattr(component, "info", None)
    if info is None:
        return None

    for key in (
        "optical_length_um",
        "optical_path_length_um",
        "optical_path_length",
        "length",
        "path_length_um",
        "path_length",
        "wg_length_um",
        "wg_length",
        "route_info_length",
        "route_info_strip_length",
    ):
        raw_length = info.get(key) if isinstance(info, dict) else getattr(info, key, None)
        resolved = _coerce_nonnegative_float(raw_length)
        if resolved is not None:
            return resolved
    return None


def normalize_internal_delays(internal_delays_um: dict[str, Any]) -> dict[str, float]:
    normalized: dict[str, float] = {}
    for instance_name, raw_delay in internal_delays_um.items():
        if is_auto_internal_delay_value(raw_delay):
            normalized[instance_name] = 0.0
            continue
        try:
            normalized[instance_name] = float(raw_delay)
        except (TypeError, ValueError):
            normalized[instance_name] = 0.0
    return normalized


def resolve_internal_delays_for_instances(
    schematic: SchematicLike,
    internal_delays_um: dict[str, Any],
) -> dict[str, float]:
    resolved_delays = dict(internal_delays_um)
    for instance_name, instance in schematic.netlist.instances.items():
        if not isinstance(instance_name, str):
            continue
        raw_delay = resolved_delays.get(instance_name, 0.0)
        if is_auto_internal_delay_value(raw_delay):
            component_name = coerce_instance_component_name(instance)
            if component_name is None:
                resolved_delays[instance_name] = 0.0
                continue
            resolved_delay = component_internal_delay_um(component_name)
            resolved_delays[instance_name] = resolved_delay if resolved_delay is not None else 0.0
            continue

        try:
            resolved_delays[instance_name] = float(raw_delay)
        except (TypeError, ValueError):
            resolved_delays[instance_name] = 0.0

    return resolved_delays


def load_benchmark_metadata(
    benchmark_name: str,
    schematic: SchematicLike | None = None,
    *,
    schematic_loader: Callable[[str], SchematicLike] | None = None,
) -> dict[str, Any]:
    """Load optional benchmark metadata used by path-length analysis."""
    benchmark_module = importlib.import_module(f"benchmarks.{benchmark_name}")
    node_types = getattr(benchmark_module, "NODE_TYPES", {})
    internal_delays_um = dict(getattr(benchmark_module, "INTERNAL_DELAYS_UM", {}))

    if schematic is None and schematic_loader is not None:
        try:
            schematic = schematic_loader(benchmark_name)
        except Exception:
            schematic = None
    if schematic is None:
        build_schematic = getattr(benchmark_module, "build_schematic", None)
        if callable(build_schematic):
            try:
                built_schematic = build_schematic()
                schematic = cast(SchematicLike, built_schematic)
            except Exception:
                schematic = None

    if schematic is not None:
        internal_delays_um = resolve_internal_delays_for_instances(
            schematic,
            internal_delays_um,
        )
    else:
        internal_delays_um = normalize_internal_delays(internal_delays_um)

    return {
        "node_types": node_types,
        "internal_delays_um": internal_delays_um,
    }
