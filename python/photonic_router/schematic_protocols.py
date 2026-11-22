"""What the analyses read of a schematic, as structural types.

A gdsfactory `Schematic` satisfies both; so does any object with the
attributes, which is what the tests build.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol


class SchematicLike(Protocol):
    """A schematic read for its ``netlist`` only.

    ``netlist.instances`` maps an instance name to an instance whose
    ``component`` is the cell's name or an object with a ``name``;
    ``netlist.routes`` maps a net name to a bundle whose ``links`` maps one
    ``"instance,port"`` string to another.
    """

    netlist: Any


class PlacedSchematicLike(SchematicLike, Protocol):
    """A schematic read for its netlist and its placements.

    ``placements`` maps an instance name to an object with ``x`` and ``y``.
    """

    placements: Mapping[str, Any]
