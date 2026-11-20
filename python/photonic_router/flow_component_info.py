"""Shared gdsfactory Component info helpers for routing flow stages."""

from __future__ import annotations

from typing import Any

from gdsfactory.component import Component


def component_info(component: Component) -> Any:
    """Return the component's mutable info object."""
    return component.info


def copy_component_info(source: Component, target: Component) -> None:
    source_info = getattr(source, "info", None)
    if source_info is None:
        return
    target_info = component_info(target)
    for key, value in getattr(source_info, "items", lambda: ())():
        if key not in target_info:
            target_info[key] = value
