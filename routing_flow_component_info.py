"""Shared gdsfactory Component info helpers for routing flow stages."""

from typing import Any

from gdsfactory.component import Component


def component_info(component: Component) -> Any:
    """Return a mutable component info object, creating one for test doubles."""
    info = getattr(component, "info", None)
    if info is None:
        info = {}
        setattr(component, "info", info)
    return info


def copy_component_info(source: Component, target: Component) -> None:
    source_info = getattr(source, "info", None)
    if source_info is None:
        return
    target_info = component_info(target)
    for key, value in getattr(source_info, "items", lambda: ())():
        if key not in target_info:
            target_info[key] = value
