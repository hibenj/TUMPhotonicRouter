"""Debug probe SVG and failure artifact writing for the Rust-backed photonic router."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path


def ensure_dir(path: Path) -> None:
    """Create the directory and its parents; an existing one is left as it is."""
    path.mkdir(parents=True, exist_ok=True)


def cells_bbox(cells: set[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    """Return ``(min x, max x, min y, max y)`` of ``cells``, or None for an empty set."""
    if not cells:
        return None
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return min(xs), max(xs), min(ys), max(ys)


def rect_cell_count(
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    """Count the cells of a rectangle with inclusive bounds; 0 if a maximum is below its minimum."""
    width = max(0, max_x - min_x + 1)
    height = max(0, max_y - min_y + 1)
    return width * height


def rect_overlap_cell_count(
    rect: tuple[int, int, int, int],
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    """Count the cells that ``rect`` shares with the rectangle given by the keyword bounds.

    ``rect`` is (min_x, min_y, max_x, max_y); all bounds are inclusive cell indices.
    Returns 0 if the two do not overlap.
    """
    rect_min_x, rect_min_y, rect_max_x, rect_max_y = rect
    overlap_min_x = max(min_x, rect_min_x)
    overlap_max_x = min(max_x, rect_max_x)
    overlap_min_y = max(min_y, rect_min_y)
    overlap_max_y = min(max_y, rect_max_y)
    return rect_cell_count(
        min_x=overlap_min_x,
        max_x=overlap_max_x,
        min_y=overlap_min_y,
        max_y=overlap_max_y,
    )


def route_cells_bbox(cells: Iterable[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    """Return the bounding box of cells as (min_x, max_x, min_y, max_y), or None without cells."""
    normalized = [(int(cell[0]), int(cell[1])) for cell in cells]
    if not normalized:
        return None
    xs = [cell[0] for cell in normalized]
    ys = [cell[1] for cell in normalized]
    return min(xs), max(xs), min(ys), max(ys)
