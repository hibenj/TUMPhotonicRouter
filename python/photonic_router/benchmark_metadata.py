"""Benchmark metadata loading and component-derived delay resolution."""

from __future__ import annotations

import importlib
from typing import Any, cast

from photonic_router.schematic_protocols import SchematicLike


def _coerce_nonnegative_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return max(0.0, float(cast(Any, value)))
    except (TypeError, ValueError):
        return None


def is_auto_internal_delay_value(value: Any) -> bool:
    """Whether an `INTERNAL_DELAYS_UM` entry asks for the component's own length.

    True for None and for one of the words `auto`, `auto-detect`,
    `auto_detect`, `auto detect`, `infer`, `infer_from_component`,
    `from_component`, `from-component` (case and surrounding whitespace
    ignored); False for every number and every other string.
    """
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
    """The component name of a netlist instance, or None when it has none.

    `instance.component` is either the name itself or an object with a
    non-empty string `name`.
    """
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
    """The optical path length a component reports about itself, in um.

    Builds the component with `gdsfactory.get_component`; when that raises,
    the generic PDK is activated and its `cell` lookup is tried instead. The
    length is the first of the known length keys of the component's `info`
    (`optical_length_um`, `optical_path_length_um`, ..., `length`,
    `route_info_length`, ...) that converts to a float; a negative value
    counts as 0.0.

    Returns None when the component cannot be built, has no `info`, or none
    of the keys holds a number. Never raises. Assumes that activating the
    generic PDK as a side effect is acceptable to the caller.
    """
    try:
        from gdsfactory import get_component

        component = get_component(component_name)
    except ValueError:
        component = None

    if component is None:
        try:
            from gdsfactory.gpdk import get_generic_pdk

            pdk = get_generic_pdk()
            pdk.activate()
            pdk_cell = getattr(pdk, "cell", None)
            component = pdk_cell(component_name) if callable(pdk_cell) else None
        except ValueError:
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
    """The declared internal delays as floats in um, without a schematic at hand.

    An entry that asks for the component's own length
    (`is_auto_internal_delay_value`) cannot be resolved here and becomes
    0.0, as does a value that does not convert to a float. Returns a new
    dict; the argument is not modified.
    """
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
    """The internal delays in um, with an entry for every instance of the schematic.

    For each instance of `schematic.netlist.instances` (string names only):
    an entry that asks for the component's own length
    (`is_auto_internal_delay_value`) is replaced by
    `component_internal_delay_um` of the instance's component, 0.0 when the
    instance names no component or the component reports no length; an
    instance without an entry gets 0.0; any other value is converted to a
    float, 0.0 when it does not convert.

    Returns a new dict; the argument is not modified. An entry whose name is
    not an instance of the schematic is passed through as it was given.
    """
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
) -> dict[str, Any]:
    """Load optional benchmark metadata used by path-length analysis.

    Without `schematic` the benchmark's own `build_schematic()` gives it;
    an error of that function is the benchmark's and is not caught.
    """
    benchmark_module = importlib.import_module(f"benchmarks.{benchmark_name}")
    node_types = getattr(benchmark_module, "NODE_TYPES", {})
    internal_delays_um = dict(getattr(benchmark_module, "INTERNAL_DELAYS_UM", {}))
    # the fan-in of the benchmark's own cells: component -> its input-side
    # ports (every other port is its output side; no ports: a coupler)
    input_ports_by_component = {
        str(component): tuple(str(port) for port in ports)
        for component, ports in dict(
            getattr(benchmark_module, "INPUT_PORTS_BY_COMPONENT", {})
        ).items()
    }
    topology_metadata = getattr(benchmark_module, "TOPOLOGY_METADATA", {})
    node_depths = getattr(benchmark_module, "NODE_DEPTHS", {})
    node_ranks = getattr(benchmark_module, "NODE_RANKS", {})
    edge_ranks = getattr(benchmark_module, "EDGE_RANKS", {})

    if schematic is None:
        build_schematic = getattr(benchmark_module, "build_schematic", None)
        if callable(build_schematic):
            schematic = cast(SchematicLike, build_schematic())

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
        "input_ports_by_component": input_ports_by_component,
        "topology_metadata": topology_metadata,
        "node_depths": node_depths,
        "node_ranks": node_ranks,
        "edge_ranks": edge_ranks,
    }
