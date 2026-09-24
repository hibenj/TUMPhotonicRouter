"""Unit tests for `python/photonic_router/primitive_library.py`.

The module's contract is a 1:1 mapping from the Rust router's primitive ids to
gdsfactory components plus metadata. The central test below reads the ids out of
the Rust library itself (`describe_primitives`) rather than hard-coding a range,
so the mapping is checked for totality against the real producer: if the Rust
library ever grows a primitive, this test fails instead of the realization
silently placing nothing.

Added by Milestone 6, Slice 3 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md` (the
module had no direct test).
"""

from __future__ import annotations

import pytest

from photonic_router.primitive_library import (
    PrimitiveLibrary,
    PrimitiveMetadata,
    get_primitive_library,
    reset_primitive_library,
)
from photonic_router.static_obstacle_builder import _load_rust_backend


@pytest.fixture(autouse=True)
def _isolated_singleton():
    """Every test starts and ends with no cached global library, so the
    caching test below cannot leak an instance into the others."""
    reset_primitive_library()
    yield
    reset_primitive_library()


def _rust_primitive_descriptions() -> list[dict]:
    backend = _load_rust_backend()
    assert backend is not None, "the Rust extension must be built for this test"
    # The smallest router that carries a default primitive library; the grid
    # size does not affect which primitives the library holds.
    router = backend.PyPhotonicRouter(
        backend.GridSpec(20, 20, 1.0, 0.0, 0.0),
        backend.PrimitiveLibraryConfig(),
        backend.AStarConfig(),
    )
    return list(router.describe_primitives())


def test_the_id_to_component_mapping_is_total_over_the_rust_primitive_ids():
    descriptions = _rust_primitive_descriptions()
    library = PrimitiveLibrary()
    missing_components = [
        description["id"]
        for description in descriptions
        if library.get_component(description["id"]) is None
    ]
    missing_metadata = [
        description["id"]
        for description in descriptions
        if library.get_metadata(description["id"]) is None
    ]
    assert missing_components == [], "every Rust primitive id needs a component"
    assert missing_metadata == [], "every Rust primitive id needs metadata"


def test_the_mapping_has_no_entries_the_rust_library_does_not_know():
    # 1:1, not merely total: the Python library must not invent ids either.
    rust_ids = {description["id"] for description in _rust_primitive_descriptions()}
    library = PrimitiveLibrary()
    assert set(library.components) == rust_ids
    assert set(library.metadata) == rust_ids


def test_the_library_holds_six_primitives_per_heading():
    # Two straights, two 45-degree turns and two 90-degree turns for each of
    # the eight headings -- 48 ids, which is also what the Rust library
    # reports (checked against it in the two tests above).
    library = PrimitiveLibrary()
    assert len(library.components) == 48
    per_start_angle: dict[int, int] = {}
    for metadata in library.metadata.values():
        per_start_angle[metadata.start_angle] = per_start_angle.get(metadata.start_angle, 0) + 1
    assert per_start_angle == {angle: 6 for angle in range(8)}


def test_each_metadata_entry_records_the_heading_change_of_its_primitive():
    library = PrimitiveLibrary()
    # The six primitives of heading 0 are ids 0..5 in build order: short
    # straight, long straight, +45, -45, +90, -90.
    deltas = [
        (library.get_metadata(primitive_id).end_angle - 0) % 8 for primitive_id in range(6)
    ]
    assert deltas == [0, 0, 1, 7, 2, 6]
    # The long straight is four cells where the short one is one cell, and a
    # straight costs no bend.
    short, long = library.get_metadata(0), library.get_metadata(1)
    assert long.length_um == pytest.approx(4.0 * short.length_um)
    assert short.bend_cost == 0.0 and long.bend_cost == 0.0
    # A 90-degree turn costs twice a 45-degree one.
    assert library.get_metadata(2).bend_cost == 1.0
    assert library.get_metadata(4).bend_cost == 2.0


def test_unknown_ids_return_none_rather_than_raising():
    library = PrimitiveLibrary()
    assert library.get_component(48) is None
    assert library.get_metadata(-1) is None


def test_get_all_metadata_returns_a_copy():
    library = PrimitiveLibrary()
    snapshot = library.get_all_metadata()
    snapshot[999] = PrimitiveMetadata(0, 0, 0.0, 0.0)
    assert 999 not in library.metadata


def test_get_primitive_library_caches_the_instance():
    first = get_primitive_library()
    assert get_primitive_library() is first
    # Only the explicit reset drops the cache, and the next call rebuilds.
    reset_primitive_library()
    second = get_primitive_library()
    assert second is not first
    assert set(second.components) == set(first.components)
