"""KLayout save options shared by every GDS this repository writes."""

from __future__ import annotations

import klayout.db as kdb

# A GDSII XY record holds at most 8191 points by convention, but its 16-bit
# record-length field overflows at 4095 points (8 bytes each), and KLayout's
# default split threshold (8000) sits above that: a long, finely sampled
# meander then produces records other readers reject. Splitting at 4000
# keeps every record well inside the format.
GDS_MAX_VERTEX_COUNT = 4000


def gds_save_options() -> kdb.SaveLayoutOptions:
    options = kdb.SaveLayoutOptions()
    options.gds2_max_vertex_count = GDS_MAX_VERTEX_COUNT
    return options
