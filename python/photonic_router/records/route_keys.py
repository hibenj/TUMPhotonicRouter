"""The keys a routed connection is identified by: a port, and an edge between two ports."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PortRef:
    """One port of one instance of the netlist."""

    instance: str
    port: str


@dataclass(frozen=True)
class RoutedEdgeKey:
    """One routed connection: the net's name and the two ports it joins."""

    net_name: str
    source: PortRef
    target: PortRef
