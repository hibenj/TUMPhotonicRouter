//! Single-layer obstacle map and routing database primitives.
//!
//! The map is the source of truth for blocked cells. It stores static layout
//! obstacles separately from dynamic routed-net occupancy so routers can
//! commit, rip up, and reroute nets without rebuilding fixed layout geometry.
//!
//! # Representation: one dense record per cell
//!
//! Every in-bounds cell `(x, y)` owns one record at index `y * width + x`.
//! The record is stored column-wise, one plane per field, so that the field
//! every search step reads is one byte wide and the cold fields are only
//! allocated once something writes them:
//!
//! | plane               | type    | meaning                                              |
//! |---------------------|---------|------------------------------------------------------|
//! | `flags`             | `u8`    | [`STATIC_BIT`], [`STATIC_RECT_BIT`], [`DYNAMIC_BIT`], [`DYNAMIC_CORE_BIT`] |
//! | `static_refs`       | `u16`   | explicit static references (`add_static_cell(s)`/`add_static_keys`) |
//! | `dynamic_refs`      | `u16`   | committed routes whose blocked footprint (core + clearance halo) covers the cell |
//! | `dynamic_core_refs` | `u16`   | committed routes whose core covers the cell          |
//! | `owner`             | `NetId` | the cell's dynamic owner, [`NO_OWNER`] when none      |
//! | `history_cost`      | `u32`   | accumulated rip-up history penalty                   |
//! | `congestion_cost`   | `u32`   | accumulated soft congestion penalty                  |
//!
//! That is 15 bytes per cell for the routing database proper and 8 more once
//! history or congestion costs exist (a 19043 x 6398 map, the largest paper
//! cell, is 122 M cells: 122 MB of flags, 1.8 GB with every plane allocated).
//! `flags` is allocated with the map; every other plane is empty until its
//! first write (an empty plane reads as all zero / `NO_OWNER`) and is shared
//! copy-on-write between a map and its clones. The engine clones the whole
//! map once per net for its static-only probe and immediately calls
//! [`ObstacleMap::clear_dynamic`] on the clone, so sharing keeps that clone at
//! the cost of copying the one-byte `flags` plane.
//!
//! # Invariants
//!
//! * `STATIC_BIT` is set iff `static_refs > 0`; `DYNAMIC_BIT` iff
//!   `dynamic_refs > 0`; `DYNAMIC_CORE_BIT` iff `dynamic_core_refs > 0`.
//! * `STATIC_RECT_BIT` is set iff at least one rectangle in `static_rects`
//!   covers the cell. Rectangles are rasterised into the bit when they are
//!   added; `clear_static_rects`/`set_static_rects` clear the bit plane-wide
//!   and rasterise again. A rectangle may reach outside the map (clearance
//!   halos of boundary geometry do); only its in-bounds cells are rasterised,
//!   and `is_static_blocked` consults the rectangle list for the
//!   out-of-bounds cells so that they answer as they always did.
//! * "Is this cell free" is answered by the `flags` byte alone:
//!   [`ObstacleMap::is_blocked`] is `STATIC_BIT | STATIC_RECT_BIT | DYNAMIC_BIT`
//!   (out of bounds counts as blocked), and `check_cell_free(x, y, opened)`
//!   is "not `DYNAMIC_BIT`, and either `opened` contains the cell or neither
//!   static bit is set". `rect_blocked`, `rect_free`, `check_cells_free` and
//!   `check_primitive_footprint_free` are loops over `check_cell_free`.
//! * Owners. `owner` is the first net that committed a footprint over the
//!   cell; when that net is ripped up, the cell passes to the first other net
//!   (in `net_routes` iteration order) whose committed footprint still covers
//!   it. The inverted index "every net whose committed footprint covers the
//!   cell" is `owner` plus `co_owners[key]`, where `co_owners` is a hash map
//!   keyed by cell that only holds the second and later owners, so it is
//!   empty unless footprints overlap (clearance halos, crossings). The index
//!   mirrors `net_routes` exactly: a cell has an owner iff some committed
//!   route covers it, every covering net is listed once, and no other net is.
//!   `owner` therefore never disagrees with the index, which is why the two
//!   bookkeeping structures of the old hash-map representation (first owner
//!   and owner list) collapse into one slot plus an overflow map.
//! * Per-net cell lists (`net_routes`, `net_core_routes`) are keyed by net
//!   and unchanged. Iteration over the per-cell accessors
//!   (`static_obstacle_keys`, `dynamic_obstacle_entries`, ...) is in grid
//!   order (`y` outer, `x` inner).

use std::sync::Arc;

use rustc_hash::{FxHashMap, FxHashSet};

/// Packed integer grid-cell key.
pub type CellKey = u64;

/// Application-level net identifier.
pub type NetId = u64;

/// Owner slot value of a cell no committed route covers. Net ids are
/// application-level indices; `NetId::MAX` is never one of them.
const NO_OWNER: NetId = NetId::MAX;

/// Distance metric used when expanding blocked cells by clearance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClearanceMetric {
    /// Diamond-shaped expansion: `abs(dx) + abs(dy) <= radius`.
    Manhattan,
    /// Square-shaped expansion: `max(abs(dx), abs(dy)) <= radius`.
    Chebyshev,
}

/// Discretized footprint for a routed waveguide.
///
/// `blocked_cells` contains both core and clearance cells. `clearance_cells`
/// excludes the core cells, which is useful for diagnostics or visualization.
#[derive(Clone, Debug, Default)]
pub struct WaveguideFootprint {
    pub core_cells: FxHashSet<CellKey>,
    pub clearance_cells: FxHashSet<CellKey>,
    pub blocked_cells: FxHashSet<CellKey>,
}

/// Inclusive integer-grid rectangle for compact obstacle storage.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct GridRect {
    pub x_min: i32,
    pub y_min: i32,
    pub x_max: i32,
    pub y_max: i32,
}

impl GridRect {
    #[inline]
    fn contains(&self, x: i32, y: i32) -> bool {
        x >= self.x_min && x <= self.x_max && y >= self.y_min && y <= self.y_max
    }
}

/// Pack signed `(x, y)` grid coordinates into a compact `u64` key.
///
/// The operation preserves the full `i32` coordinate range using two's-complement
/// representation, and is reversible with [`unpack_xy`].
#[inline]
pub fn pack_xy(x: i32, y: i32) -> CellKey {
    ((x as u32 as u64) << 32) | (y as u32 as u64)
}

/// Unpack a key created by [`pack_xy`] back into signed grid coordinates.
#[inline]
pub fn unpack_xy(key: CellKey) -> (i32, i32) {
    let x = (key >> 32) as u32 as i32;
    let y = (key & 0xffff_ffff) as u32 as i32;
    (x, y)
}

/// Expand packed cells by `radius` in `metric`, clipped to a `width` x
/// `height` grid; input cells outside the grid contribute nothing. The one
/// clearance loop behind [`ObstacleMap::inflate_cells`] and
/// `static_obstacle_builder::inflate_keys`.
pub(crate) fn inflate_packed_cells<I>(
    cells: I,
    width: i32,
    height: i32,
    radius: i32,
    metric: ClearanceMetric,
) -> FxHashSet<CellKey>
where
    I: IntoIterator<Item = CellKey>,
{
    let in_bounds = |x: i32, y: i32| x >= 0 && y >= 0 && x < width && y < height;
    let mut inflated = FxHashSet::default();
    for key in cells {
        let (x, y) = unpack_xy(key);
        if !in_bounds(x, y) {
            continue;
        }
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                let inside_metric = match metric {
                    ClearanceMetric::Manhattan => dx.abs() + dy.abs() <= radius,
                    ClearanceMetric::Chebyshev => dx.abs().max(dy.abs()) <= radius,
                };
                if !inside_metric {
                    continue;
                }
                let nx = x + dx;
                let ny = y + dy;
                if in_bounds(nx, ny) {
                    inflated.insert(pack_xy(nx, ny));
                }
            }
        }
    }
    inflated
}

/// One cold field of the per-cell record: a plane that is empty (reads as
/// `none` everywhere) until its first write and that clones of the map share
/// until one side writes to it.
#[derive(Clone, Debug)]
struct Plane<T: Copy>(Arc<Vec<T>>);

impl<T: Copy> Plane<T> {
    fn empty() -> Self {
        Self(Arc::new(Vec::new()))
    }

    #[inline]
    fn is_allocated(&self) -> bool {
        !self.0.is_empty()
    }

    /// Value at `idx`, `none` while the plane is unallocated.
    #[inline]
    fn get(&self, idx: usize, none: T) -> T {
        self.0.get(idx).copied().unwrap_or(none)
    }

    /// Writable slot at `idx`; allocates the plane (`cell_count` cells of
    /// `none`) on the first write and unshares it from clones.
    #[inline]
    fn slot(&mut self, idx: usize, cell_count: usize, none: T) -> &mut T {
        let cells = Arc::make_mut(&mut self.0);
        if cells.is_empty() {
            *cells = vec![none; cell_count];
        }
        &mut cells[idx]
    }

    /// Drop every value (the plane is unallocated again).
    fn clear(&mut self) {
        self.0 = Arc::new(Vec::new());
    }

    /// `(idx, value)` of every cell whose value is not `none`, in grid order.
    fn set_cells(&self, none: T) -> impl Iterator<Item = (usize, T)> + '_
    where
        T: PartialEq,
    {
        self.0
            .iter()
            .copied()
            .enumerate()
            .filter(move |&(_, value)| value != none)
    }
}

impl Plane<u16> {
    /// Add one reference at `idx`. Returns true when the count was zero.
    fn increment(&mut self, idx: usize, cell_count: usize) -> bool {
        let count = self.slot(idx, cell_count, 0);
        let was_zero = *count == 0;
        *count = count
            .checked_add(1)
            .expect("cell reference count overflowed u16");
        was_zero
    }

    /// Remove one reference at `idx`. Returns `None` when there was none,
    /// otherwise whether the count is now zero.
    fn decrement(&mut self, idx: usize, cell_count: usize) -> Option<bool> {
        if self.get(idx, 0) == 0 {
            return None;
        }
        let count = self.slot(idx, cell_count, 0);
        *count -= 1;
        Some(*count == 0)
    }
}

/// Single-layer obstacle map for photonic waveguide routing.
///
/// Static obstacles represent fixed geometry from the Python/gdsfactory side.
/// Dynamic obstacles represent committed routed waveguides and can be ripped up
/// by net. Both use reference counts, so overlapping blockages remain blocked
/// until every owner is removed. See the module documentation for the
/// per-cell record and its invariants.
#[derive(Clone, Debug)]
pub struct ObstacleMap {
    width: i32,
    height: i32,
    /// Hot plane, one byte per cell, always allocated: the `*_BIT` flags.
    flags: Vec<u8>,
    static_refs: Plane<u16>,
    dynamic_refs: Plane<u16>,
    dynamic_core_refs: Plane<u16>,
    owner: Plane<NetId>,
    history_cost: Plane<u32>,
    congestion_cost: Plane<u32>,
    /// Second and later dynamic owners of multi-owner cells (see the module
    /// documentation); absent or empty for single-owner cells.
    co_owners: FxHashMap<CellKey, Vec<NetId>>,
    /// Compact static rectangles as given, for [`Self::static_rects`] and for
    /// their cells outside the map.
    static_rects: Vec<GridRect>,
    net_routes: FxHashMap<NetId, Vec<CellKey>>,
    net_core_routes: FxHashMap<NetId, Vec<CellKey>>,
}

impl ObstacleMap {
    /// Create an empty single-layer obstacle map with dimensions in grid cells.
    ///
    /// Valid coordinates are `0 <= x < width` and `0 <= y < height`.
    pub fn new(width: i32, height: i32) -> Self {
        assert!(width >= 0, "width must be non-negative");
        assert!(height >= 0, "height must be non-negative");
        let width_usize =
            usize::try_from(width).expect("width must fit usize after non-negative check");
        let height_usize =
            usize::try_from(height).expect("height must fit usize after non-negative check");
        let cell_count = width_usize
            .checked_mul(height_usize)
            .expect("obstacle map occupancy size overflow");

        Self {
            width,
            height,
            flags: vec![0; cell_count],
            static_refs: Plane::empty(),
            dynamic_refs: Plane::empty(),
            dynamic_core_refs: Plane::empty(),
            owner: Plane::empty(),
            history_cost: Plane::empty(),
            congestion_cost: Plane::empty(),
            co_owners: FxHashMap::default(),
            static_rects: Vec::new(),
            net_routes: FxHashMap::default(),
            net_core_routes: FxHashMap::default(),
        }
    }

    /// Map width in grid cells.
    #[inline]
    pub fn width(&self) -> i32 {
        self.width
    }

    /// Map height in grid cells.
    #[inline]
    pub fn height(&self) -> i32 {
        self.height
    }

    /// Return true if `(x, y)` is inside the map.
    #[inline]
    pub fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.width && y < self.height
    }

    /// Associated-function form of [`pack_xy`].
    #[inline]
    pub fn pack_xy(x: i32, y: i32) -> CellKey {
        pack_xy(x, y)
    }

    /// Associated-function form of [`unpack_xy`].
    #[inline]
    pub fn unpack_xy(key: CellKey) -> (i32, i32) {
        unpack_xy(key)
    }

    // ----------------------------------------------------------------------
    // Static obstacles
    // ----------------------------------------------------------------------

    /// Add one static obstacle reference. Returns false if the cell is out of bounds.
    pub fn add_static_cell(&mut self, x: i32, y: i32) -> bool {
        let Some(idx) = self.dense_idx(x, y) else {
            return false;
        };
        self.add_static_ref(idx);
        true
    }

    /// Add a compact static rectangle.
    pub fn add_static_rect(&mut self, rect: GridRect) {
        if let Some(rect) = normalize_rect(rect) {
            self.static_rects.push(rect);
            self.rasterize_static_rect(rect);
        }
    }

    /// Add compact static rectangles.
    pub fn add_static_rects(&mut self, rects: &[GridRect]) {
        for rect in rects.iter().copied().filter_map(normalize_rect) {
            self.static_rects.push(rect);
            self.rasterize_static_rect(rect);
        }
    }

    /// Add many static obstacle references. Returns the number of in-bounds cells added.
    pub fn add_static_cells(&mut self, cells: &[(i32, i32)]) -> usize {
        let mut added = 0;
        for &(x, y) in cells {
            if self.add_static_cell(x, y) {
                added += 1;
            }
        }
        added
    }

    /// Add many packed static obstacle references. Returns the number of in-bounds cells added.
    pub fn add_static_keys(&mut self, keys: &FxHashSet<CellKey>) -> usize {
        let mut added = 0;
        for &key in keys {
            let (x, y) = unpack_xy(key);
            if self.add_static_cell(x, y) {
                added += 1;
            }
        }
        added
    }

    /// Remove all compact static rectangles.
    pub fn clear_static_rects(&mut self) {
        self.static_rects.clear();
        for cell in &mut self.flags {
            *cell &= !STATIC_RECT_BIT;
        }
    }

    /// Replace all compact static rectangles.
    pub fn set_static_rects(&mut self, rects: &[GridRect]) {
        self.clear_static_rects();
        self.add_static_rects(rects);
    }

    /// Compact static rectangles currently used by the map.
    pub fn static_rects(&self) -> &[GridRect] {
        &self.static_rects
    }

    /// Packed cell keys with explicit static obstacle references (rectangle
    /// cells are not listed), in grid order.
    pub fn static_obstacle_keys(&self) -> impl Iterator<Item = CellKey> + '_ {
        self.static_refs
            .set_cells(0)
            .map(move |(idx, _)| self.key_of_idx(idx))
    }

    /// Packed cell keys with dynamic route obstacle references, in grid order.
    pub fn dynamic_obstacle_keys(&self) -> impl Iterator<Item = CellKey> + '_ {
        self.dynamic_refs
            .set_cells(0)
            .map(move |(idx, _)| self.key_of_idx(idx))
    }

    /// Packed dynamic route obstacle cells with their reference counts, in grid order.
    pub fn dynamic_obstacle_entries(&self) -> impl Iterator<Item = (CellKey, u16)> + '_ {
        self.dynamic_refs
            .set_cells(0)
            .map(move |(idx, refs)| (self.key_of_idx(idx), refs))
    }

    /// Packed dynamic route core cells with their reference counts, in grid order.
    pub fn dynamic_core_obstacle_entries(&self) -> impl Iterator<Item = (CellKey, u16)> + '_ {
        self.dynamic_core_refs
            .set_cells(0)
            .map(move |(idx, refs)| (self.key_of_idx(idx), refs))
    }

    /// Packed cell keys with accumulated rip-up history costs, in grid order.
    pub fn history_entries(&self) -> impl Iterator<Item = (CellKey, u32)> + '_ {
        self.history_cost
            .set_cells(0)
            .map(move |(idx, cost)| (self.key_of_idx(idx), cost))
    }

    /// Packed cell keys with accumulated soft congestion costs, in grid order.
    pub fn congestion_entries(&self) -> impl Iterator<Item = (CellKey, u32)> + '_ {
        self.congestion_cost
            .set_cells(0)
            .map(move |(idx, cost)| (self.key_of_idx(idx), cost))
    }

    /// True when some cell carries a rip-up history cost (cheap pre-check
    /// for a window scan over [`Self::get_history_cost`]).
    pub(crate) fn has_history_costs(&self) -> bool {
        self.history_cost.is_allocated()
    }

    /// True when some cell carries a soft congestion cost.
    pub(crate) fn has_congestion_costs(&self) -> bool {
        self.congestion_cost.is_allocated()
    }

    /// Remove every static obstacle entry (compact + cell-based).
    pub fn clear_static_cells(&mut self) {
        self.static_refs.clear();
        self.static_rects.clear();
        for cell in &mut self.flags {
            *cell &= !(STATIC_BIT | STATIC_RECT_BIT);
        }
    }

    /// Remove one static obstacle reference. Returns true when a reference was removed.
    pub fn remove_static_cell(&mut self, x: i32, y: i32) -> bool {
        let Some(idx) = self.dense_idx(x, y) else {
            return false;
        };
        self.remove_static_ref(idx)
    }

    /// Remove many packed static obstacle references.
    ///
    /// This decrements the same reference counters used by [`Self::add_static_keys`],
    /// so derived reservations can be removed without clearing unrelated static
    /// geometry that happens to share a cell.
    pub fn remove_static_keys(&mut self, keys: &FxHashSet<CellKey>) -> usize {
        let mut removed_count = 0;
        for &key in keys {
            let (x, y) = unpack_xy(key);
            if self.remove_static_cell(x, y) {
                removed_count += 1;
            }
        }
        removed_count
    }

    /// Return true if a static obstacle blocks this cell.
    ///
    /// Out-of-bounds cells are not static obstacles unless a compact static
    /// rectangle reaches them; they are considered unavailable by
    /// [`Self::is_blocked`] and free-space checks regardless.
    pub fn is_static_blocked(&self, x: i32, y: i32) -> bool {
        match self.dense_idx(x, y) {
            Some(idx) => self.flags[idx] & (STATIC_BIT | STATIC_RECT_BIT) != 0,
            None => self.static_rects.iter().any(|rect| rect.contains(x, y)),
        }
    }

    /// Return true if a committed route blocks this in-bounds cell.
    pub fn is_dynamic_blocked(&self, x: i32, y: i32) -> bool {
        self.read_flag(x, y, DYNAMIC_BIT)
    }

    /// Return true if a committed route core blocks this in-bounds cell.
    pub fn is_dynamic_core_blocked(&self, x: i32, y: i32) -> bool {
        self.read_flag(x, y, DYNAMIC_CORE_BIT)
    }

    /// Return true if the cell is unavailable for routing.
    ///
    /// Out-of-bounds cells are treated as blocked.
    pub fn is_blocked(&self, x: i32, y: i32) -> bool {
        match self.dense_idx(x, y) {
            Some(idx) => self.flags[idx] & (STATIC_BIT | STATIC_RECT_BIT | DYNAMIC_BIT) != 0,
            None => true,
        }
    }

    /// Total static plus dynamic reference count for an in-bounds cell.
    pub fn ref_count(&self, x: i32, y: i32) -> u32 {
        let Some(idx) = self.dense_idx(x, y) else {
            return 0;
        };
        u32::from(self.static_refs.get(idx, 0)) + u32::from(self.dynamic_refs.get(idx, 0))
    }

    // ----------------------------------------------------------------------
    // Dynamic obstacles: commit and rip-up
    // ----------------------------------------------------------------------

    /// Commit a routed waveguide for `net_id`.
    ///
    /// If this net already has a committed route, the old route is ripped up
    /// after the new cells pass bounds validation. Duplicate cells inside one
    /// route are stored once.
    ///
    /// Returns false and leaves the map unchanged if any cell is out of bounds.
    pub fn commit_route(&mut self, net_id: NetId, route_cells: &[(i32, i32)]) -> bool {
        let mut keys = Vec::with_capacity(route_cells.len());
        let mut seen = FxHashSet::default();

        for &(x, y) in route_cells {
            if !self.in_bounds(x, y) {
                return false;
            }

            let key = pack_xy(x, y);
            if seen.insert(key) {
                keys.push(key);
            }
        }

        let old_route_keys: FxHashSet<CellKey> = self
            .net_routes
            .get(&net_id)
            .map(|route| route.iter().copied().collect())
            .unwrap_or_default();
        for &key in &keys {
            let existing_refs = self.dynamic_refs_at_key(key);
            let same_net_refs = u16::from(old_route_keys.contains(&key));
            if existing_refs > same_net_refs {
                return false;
            }
        }

        self.ripup_route(net_id);

        for &key in &keys {
            let idx = self.idx_of_key(key);
            self.add_dynamic_ref(idx);
            self.add_dynamic_core_ref(idx);
            self.add_owner(idx, key, net_id);
        }
        self.net_core_routes.insert(net_id, keys.clone());
        self.net_routes.insert(net_id, keys);
        true
    }

    /// Commit a route with separate core and clearance cells.
    ///
    /// The core footprint must not overlap another committed dynamic obstacle.
    /// Clearance cells may overlap other clearance reservations: the search has
    /// already checked the new core against existing reservations.
    pub fn commit_route_with_clearance_overlap(
        &mut self,
        net_id: NetId,
        core_cells: &[(i32, i32)],
        blocked_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
    ) -> bool {
        self.commit_route_with_clearance_and_allowed_core_overlaps(
            net_id,
            core_cells,
            blocked_cells,
            clearance_exempt_cells,
            &FxHashSet::default(),
        )
    }

    /// Commit a route while allowing core overlaps with selected existing nets.
    ///
    /// This is used as a first crossing-aware baseline: search may temporarily
    /// open expected partner routes, and commit validates that any resulting
    /// dynamic core overlap belongs only to those expected partners.
    pub fn commit_route_with_clearance_and_allowed_core_overlaps(
        &mut self,
        net_id: NetId,
        core_cells: &[(i32, i32)],
        blocked_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
        allowed_core_overlap_nets: &FxHashSet<NetId>,
    ) -> bool {
        self.commit_route_with_clearance_and_allowed_core_overlap_cells(
            net_id,
            core_cells,
            blocked_cells,
            clearance_exempt_cells,
            allowed_core_overlap_nets,
            None,
        )
    }

    /// Check phase of [`Self::commit_route_with_clearance_and_allowed_core_overlap_cells`]:
    /// bounds, dedup and the per-core-cell occupancy rule (an occupied cell is
    /// only acceptable as the net's own previous route, a clearance-exempt
    /// cell without foreign core, or an allowed crossing overlap). Returns the
    /// prepared `(blocked_keys, core_keys)` when the commit would succeed.
    /// This is the ONE implementation of the rule: `commit_` calls it and then
    /// mutates, `can_commit_` returns only the verdict -- callers that used to
    /// clone the whole map for a trial commit (endpoint correction, 14 % of a
    /// multiportmmi_32x32 run) use `can_commit_` instead.
    fn plan_commit_route_with_clearance_and_allowed_core_overlap_cells(
        &self,
        net_id: NetId,
        core_cells: &[(i32, i32)],
        blocked_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
        allowed_core_overlap_nets: &FxHashSet<NetId>,
        allowed_core_overlap_cells: Option<&FxHashSet<CellKey>>,
    ) -> Option<(Vec<CellKey>, Vec<CellKey>)> {
        let mut keys = Vec::with_capacity(blocked_cells.len());
        let mut seen = FxHashSet::default();
        for &(x, y) in blocked_cells {
            if !self.in_bounds(x, y) {
                return None;
            }

            let key = pack_xy(x, y);
            if seen.insert(key) {
                keys.push(key);
            }
        }

        let old_route_keys: FxHashSet<CellKey> = self
            .net_routes
            .get(&net_id)
            .map(|route| route.iter().copied().collect())
            .unwrap_or_default();
        let old_core_route_keys: FxHashSet<CellKey> = self
            .net_core_routes
            .get(&net_id)
            .map(|route| route.iter().copied().collect())
            .unwrap_or_default();
        let clearance_exempt_keys: FxHashSet<CellKey> = clearance_exempt_cells
            .iter()
            .filter_map(|&(x, y)| self.in_bounds(x, y).then_some(pack_xy(x, y)))
            .collect();
        let mut core_keys = Vec::with_capacity(core_cells.len());
        let mut seen_core = FxHashSet::default();
        for &(x, y) in core_cells {
            if !self.in_bounds(x, y) {
                return None;
            }

            let key = pack_xy(x, y);
            if !seen_core.insert(key) {
                continue;
            }
            core_keys.push(key);

            let idx = self.idx_of_key(key);
            let existing_refs = self.dynamic_refs.get(idx, 0);
            let same_net_refs = u16::from(old_route_keys.contains(&key));
            let existing_core_refs = self.dynamic_core_refs.get(idx, 0);
            let same_net_core_refs = u16::from(old_core_route_keys.contains(&key));
            let other_core_refs = existing_core_refs.saturating_sub(same_net_core_refs);
            let allowed_clearance_overlap =
                clearance_exempt_keys.contains(&key) && other_core_refs == 0;
            let allowed_crossing_overlap = if allowed_core_overlap_nets.is_empty() {
                false
            } else {
                let other_owners = self.dynamic_owners_for_key(key, Some(net_id));
                !other_owners.is_empty()
                    && other_owners
                        .iter()
                        .all(|owner| allowed_core_overlap_nets.contains(owner))
                    && allowed_core_overlap_cells.is_none_or(|cells| cells.contains(&key))
            };
            if existing_refs > same_net_refs
                && !allowed_clearance_overlap
                && !allowed_crossing_overlap
            {
                return None;
            }
        }
        Some((keys, core_keys))
    }

    /// Verdict-only twin of
    /// [`Self::commit_route_with_clearance_and_allowed_core_overlap_cells`]:
    /// true iff that commit would succeed, without touching the map.
    pub fn can_commit_route_with_clearance_and_allowed_core_overlap_cells(
        &self,
        net_id: NetId,
        core_cells: &[(i32, i32)],
        blocked_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
        allowed_core_overlap_nets: &FxHashSet<NetId>,
        allowed_core_overlap_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        self.plan_commit_route_with_clearance_and_allowed_core_overlap_cells(
            net_id,
            core_cells,
            blocked_cells,
            clearance_exempt_cells,
            allowed_core_overlap_nets,
            allowed_core_overlap_cells,
        )
        .is_some()
    }

    pub fn commit_route_with_clearance_and_allowed_core_overlap_cells(
        &mut self,
        net_id: NetId,
        core_cells: &[(i32, i32)],
        blocked_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
        allowed_core_overlap_nets: &FxHashSet<NetId>,
        allowed_core_overlap_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        let Some((keys, core_keys)) = self
            .plan_commit_route_with_clearance_and_allowed_core_overlap_cells(
                net_id,
                core_cells,
                blocked_cells,
                clearance_exempt_cells,
                allowed_core_overlap_nets,
                allowed_core_overlap_cells,
            )
        else {
            return false;
        };

        self.ripup_route(net_id);

        for &key in &keys {
            let idx = self.idx_of_key(key);
            self.add_dynamic_ref(idx);
            self.add_owner(idx, key, net_id);
        }
        for &key in &core_keys {
            let idx = self.idx_of_key(key);
            self.add_dynamic_core_ref(idx);
        }
        self.net_core_routes.insert(net_id, core_keys);
        self.net_routes.insert(net_id, keys);
        true
    }

    /// Rip up a previously committed route. Returns true when a route existed.
    pub fn ripup_route(&mut self, net_id: NetId) -> bool {
        let Some(keys) = self.net_routes.remove(&net_id) else {
            return false;
        };

        for key in keys {
            let idx = self.idx_of_key(key);
            self.remove_dynamic_ref(idx);
            self.remove_owner(idx, key, net_id);
        }
        if let Some(core_keys) = self.net_core_routes.remove(&net_id) {
            for key in core_keys {
                let idx = self.idx_of_key(key);
                self.remove_dynamic_core_ref(idx);
            }
        }
        true
    }

    /// Clear every dynamic route and routed-net owner entry.
    pub fn clear_dynamic(&mut self) {
        self.dynamic_refs.clear();
        self.dynamic_core_refs.clear();
        self.owner.clear();
        self.co_owners.clear();
        self.net_routes.clear();
        self.net_core_routes.clear();
        for cell in &mut self.flags {
            *cell &= !(DYNAMIC_BIT | DYNAMIC_CORE_BIT);
        }
    }

    /// Clear dynamic clearance reservations in `cells`, keeping dynamic core
    /// cells blocked. This is used for port access where neighboring route
    /// clearance may overlap but core geometry must remain hard blockage.
    pub fn clear_dynamic_clearance_in_cells(&mut self, cells: &[(i32, i32)]) {
        for &(x, y) in cells {
            let Some(idx) = self.dense_idx(x, y) else {
                continue;
            };
            if self.flags[idx] & DYNAMIC_CORE_BIT != 0 {
                continue;
            }
            self.drop_dynamic_refs(idx);
        }
    }

    /// Return the packed cells owned by `net_id`, if that net has a committed route.
    pub fn get_net_cells(&self, net_id: NetId) -> Option<&[CellKey]> {
        self.net_routes.get(&net_id).map(Vec::as_slice)
    }

    /// Return the packed core cells owned by `net_id`, if that net has a committed route.
    pub fn get_net_core_cells(&self, net_id: NetId) -> Option<&[CellKey]> {
        self.net_core_routes.get(&net_id).map(Vec::as_slice)
    }

    /// Return committed route cells grouped by net id.
    pub fn net_route_entries(&self) -> impl Iterator<Item = (NetId, &[CellKey])> + '_ {
        self.net_routes
            .iter()
            .map(|(&net_id, cells)| (net_id, cells.as_slice()))
    }

    /// Return committed core route cells grouped by net id.
    pub fn net_core_route_entries(&self) -> impl Iterator<Item = (NetId, &[CellKey])> + '_ {
        self.net_core_routes
            .iter()
            .map(|(&net_id, cells)| (net_id, cells.as_slice()))
    }

    // ----------------------------------------------------------------------
    // Dynamic owners
    // ----------------------------------------------------------------------

    /// Return the committed dynamic-route owner of a cell, if any.
    pub fn dynamic_owner_at(&self, x: i32, y: i32) -> Option<NetId> {
        let idx = self.dense_idx(x, y)?;
        let owner = self.owner.get(idx, NO_OWNER);
        if owner != NO_OWNER {
            return Some(owner);
        }
        let key = pack_xy(x, y);
        self.net_routes
            .iter()
            .find_map(|(&net_id, cells)| cells.contains(&key).then_some(net_id))
    }

    /// Return all committed dynamic-route owners of a cell.
    pub fn dynamic_owners_at(&self, x: i32, y: i32) -> FxHashSet<NetId> {
        if !self.in_bounds(x, y) {
            return FxHashSet::default();
        }
        self.dynamic_owners_for_key(pack_xy(x, y), None)
    }

    /// Return the committed dynamic core-route owner of a cell, if any.
    pub fn dynamic_core_owner_at(&self, x: i32, y: i32) -> Option<NetId> {
        if !self.in_bounds(x, y) {
            return None;
        }
        let key = pack_xy(x, y);
        self.net_core_routes
            .iter()
            .find_map(|(&net_id, cells)| cells.contains(&key).then_some(net_id))
    }

    /// Return all dynamic-route owners intersecting the provided cells.
    pub fn dynamic_owners_for_cells(&self, cells: &[(i32, i32)]) -> FxHashSet<NetId> {
        let mut owners = FxHashSet::default();
        for &(x, y) in cells {
            if !self.in_bounds(x, y) {
                continue;
            }
            self.extend_with_owners(pack_xy(x, y), None, &mut owners);
        }
        owners
    }

    fn dynamic_owners_for_key(
        &self,
        key: CellKey,
        exclude_net_id: Option<NetId>,
    ) -> FxHashSet<NetId> {
        let mut owners = FxHashSet::default();
        self.extend_with_owners(key, exclude_net_id, &mut owners);
        owners
    }

    /// Add every owner of the in-bounds `key` (primary slot plus co-owners)
    /// except `exclude_net_id` to `owners`.
    fn extend_with_owners(
        &self,
        key: CellKey,
        exclude_net_id: Option<NetId>,
        owners: &mut FxHashSet<NetId>,
    ) {
        let owner = self.owner.get(self.idx_of_key(key), NO_OWNER);
        if owner == NO_OWNER {
            return;
        }
        if exclude_net_id != Some(owner) {
            owners.insert(owner);
        }
        if let Some(co_owners) = self.co_owners.get(&key) {
            owners.extend(
                co_owners
                    .iter()
                    .copied()
                    .filter(|&net_id| exclude_net_id != Some(net_id)),
            );
        }
    }

    /// Register `net_id` as an owner of the cell: into the `owner` slot when
    /// the cell has none, otherwise into `co_owners` (once).
    fn add_owner(&mut self, idx: usize, key: CellKey, net_id: NetId) {
        debug_assert_ne!(net_id, NO_OWNER, "NetId::MAX is the empty-owner sentinel");
        let cell_count = self.flags.len();
        let owner = self.owner.slot(idx, cell_count, NO_OWNER);
        if *owner == NO_OWNER {
            *owner = net_id;
        } else if *owner != net_id {
            let co_owners = self.co_owners.entry(key).or_default();
            if !co_owners.contains(&net_id) {
                co_owners.push(net_id);
            }
        }
    }

    /// Remove `net_id` as an owner of the cell. When it held the `owner`
    /// slot, the slot passes to the first other covering net in `net_routes`
    /// iteration order (`net_id`'s route has already been removed from it).
    fn remove_owner(&mut self, idx: usize, key: CellKey, net_id: NetId) {
        let cell_count = self.flags.len();
        if self.owner.get(idx, NO_OWNER) != net_id {
            if let Some(co_owners) = self.co_owners.get_mut(&key) {
                if let Some(pos) = co_owners.iter().position(|&owner| owner == net_id) {
                    co_owners.swap_remove(pos);
                }
                if co_owners.is_empty() {
                    self.co_owners.remove(&key);
                }
            }
            return;
        }

        let Some(co_owners) = self.co_owners.get_mut(&key) else {
            *self.owner.slot(idx, cell_count, NO_OWNER) = NO_OWNER;
            return;
        };
        let successor = if co_owners.len() == 1 {
            co_owners[0]
        } else {
            self.net_routes
                .keys()
                .copied()
                .find(|other| co_owners.contains(other))
                .expect("every co-owner has a committed route")
        };
        let pos = co_owners
            .iter()
            .position(|&owner| owner == successor)
            .expect("successor is a co-owner");
        co_owners.swap_remove(pos);
        if co_owners.is_empty() {
            self.co_owners.remove(&key);
        }
        *self.owner.slot(idx, cell_count, NO_OWNER) = successor;
    }

    /// Full O(n) check that the owner slot plus `co_owners` exactly mirror
    /// `net_routes`.
    #[cfg(test)]
    fn owner_index_is_consistent(&self) -> bool {
        let mut expected: FxHashMap<CellKey, FxHashSet<NetId>> = FxHashMap::default();
        for (&net_id, cells) in &self.net_routes {
            for &key in cells {
                expected.entry(key).or_default().insert(net_id);
            }
        }

        let mut owned_cells = 0usize;
        for idx in 0..self.flags.len() {
            let key = self.key_of_idx(idx);
            let owner = self.owner.get(idx, NO_OWNER);
            let co_owners = self.co_owners.get(&key);
            if owner == NO_OWNER {
                if co_owners.is_some() {
                    return false;
                }
                continue;
            }
            owned_cells += 1;
            let Some(expected_owners) = expected.get(&key) else {
                return false;
            };
            let mut actual: FxHashSet<NetId> = FxHashSet::default();
            actual.insert(owner);
            for &co_owner in co_owners.map(Vec::as_slice).unwrap_or(&[]) {
                if !actual.insert(co_owner) {
                    return false;
                }
            }
            if actual != *expected_owners {
                return false;
            }
        }
        owned_cells == expected.len()
    }

    /// Clear dynamic blocking for cells owned only by the selected nets.
    ///
    /// This is intended for cloned search maps. It leaves per-net route records
    /// untouched, so callers should not use it on the committed routing DB.
    pub fn clear_dynamic_blocking_for_nets(&mut self, net_ids: &FxHashSet<NetId>) -> usize {
        if net_ids.is_empty() {
            return 0;
        }
        let mut candidate_keys = FxHashSet::default();
        for net_id in net_ids {
            if let Some(route_cells) = self.net_routes.get(net_id) {
                candidate_keys.extend(route_cells.iter().copied());
            }
        }

        let mut cleared = 0usize;
        for key in candidate_keys {
            let owners = self.dynamic_owners_for_key(key, None);
            if owners.is_empty() || !owners.iter().all(|owner| net_ids.contains(owner)) {
                continue;
            }
            let idx = self.idx_of_key(key);
            self.drop_dynamic_refs(idx);
            self.drop_dynamic_core_refs(idx);
            cleared += 1;
        }
        cleared
    }

    pub fn clear_dynamic_blocking_in_cells_for_nets(
        &mut self,
        cells: &FxHashSet<CellKey>,
        net_ids: &FxHashSet<NetId>,
    ) -> usize {
        if cells.is_empty() || net_ids.is_empty() {
            return 0;
        }
        let mut cleared = 0usize;
        for &key in cells {
            let (x, y) = unpack_xy(key);
            let Some(idx) = self.dense_idx(x, y) else {
                continue;
            };
            let owners = self.dynamic_owners_for_key(key, None);
            if owners.is_empty() || !owners.iter().all(|owner| net_ids.contains(owner)) {
                continue;
            }
            self.drop_dynamic_refs(idx);
            self.drop_dynamic_core_refs(idx);
            cleared += 1;
        }
        cleared
    }

    /// Return a copy whose dynamic obstacles are expanded by `radius_cells`.
    ///
    /// This is intended for route search. Committed routes are already stored
    /// with their own blockage radius; expanding them by the candidate route's
    /// radius makes centerline search reject paths whose future committed
    /// footprint would overlap existing routed footprints.
    pub fn clone_with_expanded_dynamic_obstacles(&self, radius_cells: i32) -> Self {
        if radius_cells <= 0 {
            return self.clone();
        }

        let mut expanded = Self {
            width: self.width,
            height: self.height,
            flags: self.flags.clone(),
            static_refs: self.static_refs.clone(),
            dynamic_refs: self.dynamic_refs.clone(),
            dynamic_core_refs: self.dynamic_core_refs.clone(),
            // Expanded search maps only need dynamic occupancy, not ownership
            // or per-net route records. Keeping these empty avoids copying and
            // updating large owner maps on every normal route.
            owner: Plane::empty(),
            history_cost: self.history_cost.clone(),
            congestion_cost: self.congestion_cost.clone(),
            co_owners: FxHashMap::default(),
            static_rects: self.static_rects.clone(),
            net_routes: FxHashMap::default(),
            net_core_routes: FxHashMap::default(),
        };
        for key in self.dynamic_obstacle_keys() {
            let (x, y) = unpack_xy(key);
            for dx in -radius_cells..=radius_cells {
                for dy in -radius_cells..=radius_cells {
                    if let Some(idx) = expanded.dense_idx(x + dx, y + dy) {
                        expanded.add_dynamic_ref(idx);
                    }
                }
            }
        }
        expanded
    }

    // ----------------------------------------------------------------------
    // Free-space queries
    // ----------------------------------------------------------------------

    /// Return true when all cells in `rect` are free for routing.
    pub fn rect_free(&self, rect: GridRect, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        !self.rect_blocked(rect, opened_cells)
    }

    /// Return true when any cell in `rect` is blocked for routing.
    ///
    /// `opened_cells` allows exact-cell openings for the current net.
    pub fn rect_blocked(&self, rect: GridRect, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        let Some(rect) = normalize_rect(rect) else {
            return false;
        };
        if !self.in_bounds(rect.x_min, rect.y_min) || !self.in_bounds(rect.x_max, rect.y_max) {
            return true;
        }

        for y in rect.y_min..=rect.y_max {
            for x in rect.x_min..=rect.x_max {
                if !self.check_cell_free(x, y, opened_cells) {
                    return true;
                }
            }
        }
        false
    }

    /// Add a rip-up history penalty to an in-bounds cell.
    ///
    /// Returns false if the cell is out of bounds. Cost addition saturates at
    /// `u32::MAX`.
    pub fn add_history_cost(&mut self, x: i32, y: i32, amount: u32) -> bool {
        let Some(idx) = self.dense_idx(x, y) else {
            return false;
        };
        let cell_count = self.flags.len();
        let entry = self.history_cost.slot(idx, cell_count, 0);
        *entry = entry.saturating_add(amount);
        true
    }

    /// Return the rip-up history cost for a cell, or zero if none exists.
    pub fn get_history_cost(&self, x: i32, y: i32) -> u32 {
        match self.dense_idx(x, y) {
            Some(idx) => self.history_cost.get(idx, 0),
            None => 0,
        }
    }

    /// Clear all rip-up history penalties.
    pub fn clear_history(&mut self) {
        self.history_cost.clear();
    }

    /// Add a soft congestion penalty to an in-bounds cell.
    ///
    /// Returns false if the cell is out of bounds. Cost addition saturates at
    /// `u32::MAX`.
    pub fn add_congestion_cost(&mut self, x: i32, y: i32, amount: u32) -> bool {
        let Some(idx) = self.dense_idx(x, y) else {
            return false;
        };
        let cell_count = self.flags.len();
        let entry = self.congestion_cost.slot(idx, cell_count, 0);
        *entry = entry.saturating_add(amount);
        true
    }

    /// Return the soft congestion cost for a cell, or zero if none exists.
    pub fn get_congestion_cost(&self, x: i32, y: i32) -> u32 {
        match self.dense_idx(x, y) {
            Some(idx) => self.congestion_cost.get(idx, 0),
            None => 0,
        }
    }

    /// Clear all soft congestion penalties.
    pub fn clear_congestion(&mut self) {
        self.congestion_cost.clear();
    }

    /// Check whether every listed cell is in bounds and free.
    ///
    /// `opened_cells` temporarily overrides static blocking for source/target
    /// port cells. Dynamic routed-net obstacles remain blocked. Out-of-bounds
    /// cells are always invalid.
    pub fn check_cells_free(
        &self,
        cells: &[(i32, i32)],
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        for &(x, y) in cells {
            if !self.check_cell_free(x, y, opened_cells) {
                return false;
            }
        }
        true
    }

    /// Check a future move primitive footprint translated by `(origin_x, origin_y)`.
    ///
    /// The footprint offsets are relative `(dx, dy)` cells. Every translated
    /// footprint cell must be in bounds and free unless explicitly opened.
    pub fn check_primitive_footprint_free(
        &self,
        origin_x: i32,
        origin_y: i32,
        footprint_offsets: &[(i32, i32)],
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        for &(dx, dy) in footprint_offsets {
            let Some(x) = origin_x.checked_add(dx) else {
                return false;
            };
            let Some(y) = origin_y.checked_add(dy) else {
                return false;
            };

            if !self.check_cell_free(x, y, opened_cells) {
                return false;
            }
        }
        true
    }

    /// Expand cells by a clearance radius using the requested metric.
    ///
    /// Returned cells are packed keys and are clipped to the map bounds.
    pub fn inflate_cells(
        &self,
        cells: &[(i32, i32)],
        clearance_radius: i32,
        metric: ClearanceMetric,
    ) -> FxHashSet<CellKey> {
        assert!(
            clearance_radius >= 0,
            "clearance_radius must be non-negative"
        );
        inflate_packed_cells(
            cells.iter().map(|&(x, y)| pack_xy(x, y)),
            self.width,
            self.height,
            clearance_radius,
            metric,
        )
    }

    /// Convert a discretized waveguide core path to core, clearance, and blocked cells.
    ///
    /// `core_path` is expected to already be rasterized onto grid cells by the
    /// Python/gdsfactory frontend or by a later Rust geometry helper.
    pub fn waveguide_path_to_blocked_cells(
        &self,
        core_path: &[(i32, i32)],
        clearance_radius: i32,
        metric: ClearanceMetric,
    ) -> WaveguideFootprint {
        let mut core_cells = FxHashSet::default();
        for &(x, y) in core_path {
            if self.in_bounds(x, y) {
                core_cells.insert(pack_xy(x, y));
            }
        }

        let blocked_cells = self.inflate_cells(core_path, clearance_radius, metric);
        let clearance_cells = blocked_cells
            .iter()
            .copied()
            .filter(|key| !core_cells.contains(key))
            .collect();

        WaveguideFootprint {
            core_cells,
            clearance_cells,
            blocked_cells,
        }
    }

    /// The one free-space rule: in bounds, no dynamic blockage, and either
    /// opened or free of static blockage (cell or rectangle).
    #[inline]
    fn check_cell_free(&self, x: i32, y: i32, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        let Some(idx) = self.dense_idx(x, y) else {
            return false;
        };
        let flags = self.flags[idx];
        if flags & DYNAMIC_BIT != 0 {
            return false;
        }
        if opened_cells.is_some_and(|opened| opened.contains(&pack_xy(x, y))) {
            return true;
        }
        flags & (STATIC_BIT | STATIC_RECT_BIT) == 0
    }

    // ----------------------------------------------------------------------
    // Record access
    // ----------------------------------------------------------------------

    /// Record index of an in-bounds cell.
    #[inline]
    fn dense_idx(&self, x: i32, y: i32) -> Option<usize> {
        if !self.in_bounds(x, y) {
            return None;
        }
        let x = usize::try_from(x).ok()?;
        let y = usize::try_from(y).ok()?;
        let width = usize::try_from(self.width).ok()?;
        y.checked_mul(width)?.checked_add(x)
    }

    /// Record index of a packed key that is known to be in bounds.
    #[inline]
    fn idx_of_key(&self, key: CellKey) -> usize {
        let (x, y) = unpack_xy(key);
        self.dense_idx(x, y)
            .expect("packed cell key of a committed or validated cell is in bounds")
    }

    /// Packed key of the cell at record index `idx`.
    #[inline]
    fn key_of_idx(&self, idx: usize) -> CellKey {
        let width = usize::try_from(self.width).expect("width is non-negative");
        let x = i32::try_from(idx % width).expect("x fits i32");
        let y = i32::try_from(idx / width).expect("y fits i32");
        pack_xy(x, y)
    }

    #[inline]
    fn read_flag(&self, x: i32, y: i32, bit: u8) -> bool {
        self.dense_idx(x, y)
            .is_some_and(|idx| self.flags[idx] & bit != 0)
    }

    /// Reference count of the dynamic (core + clearance) footprints covering
    /// an in-bounds packed key.
    #[inline]
    fn dynamic_refs_at_key(&self, key: CellKey) -> u16 {
        self.dynamic_refs.get(self.idx_of_key(key), 0)
    }

    /// Set `STATIC_RECT_BIT` on the in-bounds cells of a normalized rectangle.
    fn rasterize_static_rect(&mut self, rect: GridRect) {
        let x_max = rect.x_max.min(self.width - 1);
        let y_max = rect.y_max.min(self.height - 1);
        for y in rect.y_min..=y_max {
            for x in rect.x_min..=x_max {
                if let Some(idx) = self.dense_idx(x, y) {
                    self.flags[idx] |= STATIC_RECT_BIT;
                }
            }
        }
    }

    fn add_static_ref(&mut self, idx: usize) {
        if self.static_refs.increment(idx, self.flags.len()) {
            self.flags[idx] |= STATIC_BIT;
        }
    }

    fn remove_static_ref(&mut self, idx: usize) -> bool {
        match self.static_refs.decrement(idx, self.flags.len()) {
            None => false,
            Some(now_zero) => {
                if now_zero {
                    self.flags[idx] &= !STATIC_BIT;
                }
                true
            }
        }
    }

    fn add_dynamic_ref(&mut self, idx: usize) {
        if self.dynamic_refs.increment(idx, self.flags.len()) {
            self.flags[idx] |= DYNAMIC_BIT;
        }
    }

    fn remove_dynamic_ref(&mut self, idx: usize) {
        if self.dynamic_refs.decrement(idx, self.flags.len()) == Some(true) {
            self.flags[idx] &= !DYNAMIC_BIT;
        }
    }

    /// Forget every dynamic (core + clearance) reference of a cell.
    fn drop_dynamic_refs(&mut self, idx: usize) {
        if self.flags[idx] & DYNAMIC_BIT != 0 {
            *self.dynamic_refs.slot(idx, self.flags.len(), 0) = 0;
            self.flags[idx] &= !DYNAMIC_BIT;
        }
    }

    fn add_dynamic_core_ref(&mut self, idx: usize) {
        if self.dynamic_core_refs.increment(idx, self.flags.len()) {
            self.flags[idx] |= DYNAMIC_CORE_BIT;
        }
    }

    fn remove_dynamic_core_ref(&mut self, idx: usize) {
        if self.dynamic_core_refs.decrement(idx, self.flags.len()) == Some(true) {
            self.flags[idx] &= !DYNAMIC_CORE_BIT;
        }
    }

    /// Forget every dynamic core reference of a cell.
    fn drop_dynamic_core_refs(&mut self, idx: usize) {
        if self.flags[idx] & DYNAMIC_CORE_BIT != 0 {
            *self.dynamic_core_refs.slot(idx, self.flags.len(), 0) = 0;
            self.flags[idx] &= !DYNAMIC_CORE_BIT;
        }
    }
}

fn normalize_rect(rect: GridRect) -> Option<GridRect> {
    if rect.x_min > rect.x_max || rect.y_min > rect.y_max {
        return None;
    }
    if rect.x_min < 0 || rect.y_min < 0 {
        return None;
    }
    // A rectangle may reach past the map's width/height (clearance halos of
    // boundary geometry); `rasterize_static_rect` clips it and the list keeps
    // it as given. Query rectangles are bounds-checked by `rect_blocked`.
    Some(rect)
}

/// `flags` bit: the cell has explicit static references.
const STATIC_BIT: u8 = 1 << 0;
/// `flags` bit: a committed route's blocked footprint covers the cell.
const DYNAMIC_BIT: u8 = 1 << 1;
/// `flags` bit: a compact static rectangle covers the cell.
const STATIC_RECT_BIT: u8 = 1 << 2;
/// `flags` bit: a committed route's core covers the cell.
const DYNAMIC_CORE_BIT: u8 = 1 << 3;

#[cfg(test)]
mod tests {
    use super::*;

    /// `can_commit_...` must give exactly the verdict a trial commit on a
    /// cloned map gives (the pattern endpoint correction used, 14 % of a
    /// multiportmmi_32x32 run), and must not touch the map.
    #[test]
    fn can_commit_matches_trial_commit_verdict_and_leaves_map_untouched() {
        let mut map = ObstacleMap::new(20, 20);
        // net 1 owns a vertical at x=5 (core = blocked here, clearance 0)
        let net1: Vec<(i32, i32)> = (0..20).map(|y| (5, y)).collect();
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            1,
            &net1,
            &net1,
            &[],
            &FxHashSet::default()
        ));
        let none: FxHashSet<NetId> = FxHashSet::default();
        let cases: Vec<(&str, Vec<(i32, i32)>, Vec<(i32, i32)>, FxHashSet<NetId>)> = vec![
            (
                "free horizontal",
                (0..5).map(|x| (x, 10)).collect(),
                vec![],
                none.clone(),
            ),
            (
                "crosses net 1's core without allowance",
                (0..10).map(|x| (x, 10)).collect(),
                vec![],
                none.clone(),
            ),
            (
                "crosses net 1's core with net 1 allowed",
                (0..10).map(|x| (x, 10)).collect(),
                vec![],
                [1u64].into_iter().collect(),
            ),
            (
                "crosses net 1's core, cell clearance-exempt but foreign core",
                (0..10).map(|x| (x, 10)).collect(),
                vec![(5, 10)],
                none.clone(),
            ),
            (
                "out of bounds",
                vec![(19, 10), (20, 10)],
                vec![],
                none.clone(),
            ),
        ];
        for (label, core, exempt, allowed) in cases {
            let verdict = map.can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                2, &core, &core, &exempt, &allowed, None,
            );
            let mut trial = map.clone();
            let committed = trial.commit_route_with_clearance_and_allowed_core_overlap_cells(
                2, &core, &core, &exempt, &allowed, None,
            );
            assert_eq!(
                verdict, committed,
                "{label}: can_commit must match the trial commit"
            );
            // the check must not have registered anything
            assert!(
                !map.net_routes.contains_key(&2),
                "{label}: can_commit mutated the map"
            );
            assert!(
                !map.is_dynamic_core_blocked(0, 10),
                "{label}: can_commit mutated the map"
            );
        }
        // expected verdicts, so the case list itself is meaningful
        assert!(
            map.can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                2,
                &(0..5).map(|x| (x, 10)).collect::<Vec<_>>(),
                &(0..5).map(|x| (x, 10)).collect::<Vec<_>>(),
                &[],
                &none,
                None
            )
        );
        assert!(
            !map.can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                2,
                &(0..10).map(|x| (x, 10)).collect::<Vec<_>>(),
                &(0..10).map(|x| (x, 10)).collect::<Vec<_>>(),
                &[],
                &none,
                None
            )
        );
        assert!(
            map.can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                2,
                &(0..10).map(|x| (x, 10)).collect::<Vec<_>>(),
                &(0..10).map(|x| (x, 10)).collect::<Vec<_>>(),
                &[],
                &[1u64].into_iter().collect(),
                None
            )
        );
    }

    #[test]
    fn static_add_remove_updates_occupancy() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(!map.is_static_blocked(2, 3));
        assert!(map.add_static_cell(2, 3));
        assert!(map.is_static_blocked(2, 3));
        assert!(map.is_blocked(2, 3));
        assert!(map.remove_static_cell(2, 3));
        assert!(!map.is_static_blocked(2, 3));
        assert!(!map.is_blocked(2, 3));
    }

    #[test]
    fn static_refs_require_matching_removals() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.add_static_cell(1, 1));
        assert!(map.add_static_cell(1, 1));
        assert!(map.is_static_blocked(1, 1));
        assert!(map.remove_static_cell(1, 1));
        assert!(map.is_static_blocked(1, 1));
        assert!(map.remove_static_cell(1, 1));
        assert!(!map.is_static_blocked(1, 1));
    }

    #[test]
    fn dynamic_commit_and_ripup_update_occupancy() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route(7, &[(2, 2), (3, 2), (3, 2)]));
        assert!(map.is_dynamic_blocked(2, 2));
        assert!(map.is_dynamic_blocked(3, 2));
        assert!(map.ripup_route(7));
        assert!(!map.is_dynamic_blocked(2, 2));
        assert!(!map.is_dynamic_blocked(3, 2));
    }

    #[test]
    fn opened_cells_do_not_unblock_dynamic_obstacles() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route(7, &[(2, 2)]));
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(2, 2));

        assert!(!map.check_cells_free(&[(2, 2)], Some(&opened)));
        assert!(map.rect_blocked(
            GridRect {
                x_min: 2,
                y_min: 2,
                x_max: 2,
                y_max: 2,
            },
            Some(&opened),
        ));
    }

    #[test]
    fn commit_route_rejects_other_net_dynamic_overlap() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route(1, &[(2, 2), (3, 2)]));

        assert!(!map.commit_route(2, &[(3, 2), (4, 2)]));
        assert!(map.is_dynamic_blocked(2, 2));
        assert!(map.is_dynamic_blocked(3, 2));
        assert!(!map.is_dynamic_blocked(4, 2));
        assert!(map.get_net_cells(2).is_none());
    }

    #[test]
    fn commit_route_with_clearance_overlap_allows_halo_but_rejects_core() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route(1, &[(2, 2), (2, 3)]));

        assert!(map.commit_route_with_clearance_overlap(2, &[(4, 2)], &[(4, 2), (2, 3)], &[],));
        assert!(!map.commit_route_with_clearance_overlap(3, &[(2, 2)], &[(2, 2), (5, 5)], &[],));
        assert!(map.get_net_cells(3).is_none());
    }

    #[test]
    fn commit_route_allows_core_over_exempt_clearance_but_not_core() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route_with_clearance_overlap(1, &[(2, 2)], &[(2, 2), (3, 2)], &[],));
        assert!(map.commit_route_with_clearance_overlap(2, &[(3, 2)], &[(3, 2)], &[(3, 2)],));
        assert!(!map.commit_route_with_clearance_overlap(3, &[(2, 2)], &[(2, 2)], &[(2, 2)],));
    }

    #[test]
    fn crossing_commit_allows_expected_core_overlap_only() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route_with_clearance_overlap(1, &[(2, 2)], &[(2, 2)], &[],));
        assert!(map.commit_route_with_clearance_overlap(3, &[(4, 4)], &[(4, 4)], &[],));

        let mut allowed = FxHashSet::default();
        allowed.insert(1);
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            2,
            &[(2, 2)],
            &[(2, 2)],
            &[],
            &allowed,
        ));
        assert_eq!(map.ref_count(2, 2), 2);

        assert!(!map.commit_route_with_clearance_and_allowed_core_overlaps(
            4,
            &[(4, 4)],
            &[(4, 4)],
            &[],
            &allowed,
        ));
        assert!(map.get_net_cells(4).is_none());
    }

    #[test]
    fn cloned_search_map_can_open_expected_crossing_partner_cells() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route_with_clearance_overlap(1, &[(2, 2)], &[(2, 2), (2, 3)], &[],));
        assert!(map.commit_route_with_clearance_overlap(3, &[(4, 4)], &[(4, 4)], &[],));

        let mut expected = FxHashSet::default();
        expected.insert(1);
        let mut search_map = map.clone();
        assert_eq!(search_map.clear_dynamic_blocking_for_nets(&expected), 2);

        assert!(!search_map.is_dynamic_blocked(2, 2));
        assert!(!search_map.is_dynamic_blocked(2, 3));
        assert!(search_map.is_dynamic_blocked(4, 4));
        assert!(map.is_dynamic_blocked(2, 2));
    }

    #[test]
    fn commit_route_allows_same_net_replacement_overlap() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route(1, &[(2, 2), (3, 2)]));

        assert!(map.commit_route(1, &[(3, 2), (4, 2)]));
        assert!(!map.is_dynamic_blocked(2, 2));
        assert!(map.is_dynamic_blocked(3, 2));
        assert!(map.is_dynamic_blocked(4, 2));
    }

    #[test]
    fn dynamic_owner_index_tracks_commit_replace_and_ripup() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route(11, &[(2, 2), (3, 2)]));
        assert_eq!(map.dynamic_owner_at(2, 2), Some(11));
        assert_eq!(map.dynamic_owner_at(3, 2), Some(11));
        assert_eq!(map.dynamic_core_owner_at(2, 2), Some(11));

        let owners = map.dynamic_owners_for_cells(&[(1, 1), (2, 2), (3, 2)]);
        assert_eq!(owners.len(), 1);
        assert!(owners.contains(&11));

        assert!(map.commit_route(11, &[(4, 2)]));
        assert_eq!(map.dynamic_owner_at(2, 2), None);
        assert_eq!(map.dynamic_owner_at(4, 2), Some(11));
        assert_eq!(map.dynamic_core_owner_at(2, 2), None);
        assert_eq!(map.dynamic_core_owner_at(4, 2), Some(11));
        assert!(map.ripup_route(11));
        assert_eq!(map.dynamic_owner_at(4, 2), None);
        assert_eq!(map.dynamic_core_owner_at(4, 2), None);
    }

    #[test]
    fn dynamic_core_owner_ignores_clearance_only_cells() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route_with_clearance_overlap(11, &[(2, 2)], &[(2, 2), (2, 3)], &[],));

        assert_eq!(map.dynamic_owner_at(2, 2), Some(11));
        assert_eq!(map.dynamic_owner_at(2, 3), Some(11));
        assert_eq!(map.dynamic_core_owner_at(2, 2), Some(11));
        assert_eq!(map.dynamic_core_owner_at(2, 3), None);
    }

    #[test]
    fn clone_with_expanded_dynamic_obstacles_preserves_original_map() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route(1, &[(3, 3)]));

        let expanded = map.clone_with_expanded_dynamic_obstacles(1);
        assert!(expanded.is_dynamic_blocked(2, 2));
        assert!(expanded.is_dynamic_blocked(4, 4));
        assert!(!map.is_dynamic_blocked(2, 2));
        assert!(!map.is_dynamic_blocked(4, 4));
    }

    #[test]
    fn static_dynamic_overlap_requires_clearing_both() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.add_static_cell(4, 4));
        assert!(map.commit_route(9, &[(4, 4)]));
        assert!(map.is_blocked(4, 4));
        assert!(map.ripup_route(9));
        assert!(map.is_blocked(4, 4));
        assert!(map.remove_static_cell(4, 4));
        assert!(!map.is_blocked(4, 4));
    }

    #[test]
    fn clear_dynamic_preserves_static() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.add_static_cell(5, 5));
        assert!(map.commit_route(1, &[(5, 5), (6, 5)]));
        map.clear_dynamic();
        assert!(map.is_static_blocked(5, 5));
        assert!(map.is_blocked(5, 5));
        assert!(!map.is_dynamic_blocked(5, 5));
        assert!(!map.is_dynamic_blocked(6, 5));
        assert!(!map.is_blocked(6, 5));
    }

    #[test]
    fn out_of_bounds_behavior_unchanged() {
        let mut map = ObstacleMap::new(3, 3);
        assert!(map.is_blocked(-1, 0));
        assert!(map.is_blocked(3, 0));
        assert!(!map.is_static_blocked(-1, 0));
        assert!(!map.is_dynamic_blocked(3, 0));
        assert!(!map.add_static_cell(3, 0));
        assert!(!map.remove_static_cell(-1, 0));
    }

    #[test]
    fn compact_static_rect_blocks_cells() {
        let mut map = ObstacleMap::new(8, 8);
        map.add_static_rect(GridRect {
            x_min: 2,
            y_min: 1,
            x_max: 3,
            y_max: 4,
        });
        assert!(map.is_static_blocked(2, 1));
        assert!(map.is_static_blocked(3, 4));
        assert!(!map.is_static_blocked(1, 1));
        assert!(map.is_blocked(2, 4));
    }

    #[test]
    fn rectangle_query_rejects_when_not_opened() {
        let mut map = ObstacleMap::new(8, 8);
        map.add_static_rect(GridRect {
            x_min: 1,
            y_min: 1,
            x_max: 4,
            y_max: 2,
        });
        assert!(!map.rect_free(
            GridRect {
                x_min: 0,
                y_min: 1,
                x_max: 5,
                y_max: 1,
            },
            None
        ));
    }

    #[test]
    fn rectangle_query_allows_fully_opened_overlap() {
        let mut map = ObstacleMap::new(8, 8);
        map.add_static_rect(GridRect {
            x_min: 1,
            y_min: 1,
            x_max: 3,
            y_max: 2,
        });
        let opened: FxHashSet<CellKey> = [(1, 1), (2, 1), (3, 1), (1, 2), (2, 2), (3, 2)]
            .into_iter()
            .map(|(x, y)| pack_xy(x, y))
            .collect();
        assert!(map.rect_free(
            GridRect {
                x_min: 1,
                y_min: 1,
                x_max: 3,
                y_max: 2,
            },
            Some(&opened)
        ));
    }

    #[test]
    fn owner_index_stays_consistent_across_commit_ripup_recommit() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.owner_index_is_consistent());

        // Two overlapping routes: net 1 and net 2 share cell (3, 2).
        assert!(map.commit_route(1, &[(2, 2), (3, 2)]));
        assert!(map.owner_index_is_consistent());
        let mut allowed = FxHashSet::default();
        allowed.insert(1);
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            2,
            &[(3, 2)],
            &[(3, 2), (4, 2)],
            &[],
            &allowed,
        ));
        assert!(map.owner_index_is_consistent());

        let owners = map.dynamic_owners_at(3, 2);
        assert_eq!(owners.len(), 2);
        assert!(owners.contains(&1));
        assert!(owners.contains(&2));
        assert_eq!(map.dynamic_owners_at(2, 2), [1].into_iter().collect());
        assert_eq!(map.dynamic_owners_at(4, 2), [2].into_iter().collect());

        // Rip up net 1: cell (3, 2) should now be owned only by net 2.
        assert!(map.ripup_route(1));
        assert!(map.owner_index_is_consistent());
        assert_eq!(map.dynamic_owners_at(2, 2), FxHashSet::default());
        assert_eq!(map.dynamic_owners_at(3, 2), [2].into_iter().collect());

        // Re-commit net 2 on a disjoint set of cells: the old cells must
        // drop out of the index and the new ones must appear.
        assert!(map.commit_route(2, &[(5, 5), (6, 5)]));
        assert!(map.owner_index_is_consistent());
        assert_eq!(map.dynamic_owners_at(3, 2), FxHashSet::default());
        assert_eq!(map.dynamic_owners_at(4, 2), FxHashSet::default());
        assert_eq!(map.dynamic_owners_at(5, 5), [2].into_iter().collect());
        assert_eq!(map.dynamic_owners_at(6, 5), [2].into_iter().collect());

        // dynamic_owners_for_cells should agree with the per-cell queries.
        let combined = map.dynamic_owners_for_cells(&[(5, 5), (6, 5), (0, 0)]);
        assert_eq!(combined, [2].into_iter().collect());

        map.clear_dynamic();
        assert!(map.owner_index_is_consistent());
        assert!(map.co_owners.is_empty());
    }

    #[test]
    fn rectangle_query_rejects_partial_opening_gap() {
        let mut map = ObstacleMap::new(8, 8);
        map.add_static_rect(GridRect {
            x_min: 1,
            y_min: 1,
            x_max: 3,
            y_max: 2,
        });
        let opened: FxHashSet<CellKey> = [(1, 1), (2, 1)]
            .into_iter()
            .map(|(x, y)| pack_xy(x, y))
            .collect();
        assert!(!map.rect_free(
            GridRect {
                x_min: 1,
                y_min: 1,
                x_max: 3,
                y_max: 2,
            },
            Some(&opened)
        ));
    }

    #[test]
    fn static_rect_reaching_outside_the_map_blocks_its_in_and_out_of_bounds_cells() {
        // Clearance halos of boundary geometry produce rectangles past the
        // map edge; only in-bounds cells are rasterised, the rectangle list
        // answers for the rest, exactly as the old per-row interval index did.
        let mut map = ObstacleMap::new(8, 8);
        map.add_static_rect(GridRect {
            x_min: 6,
            y_min: 6,
            x_max: 10,
            y_max: 10,
        });
        assert!(map.is_static_blocked(7, 7));
        assert!(map.is_static_blocked(9, 9));
        assert!(!map.is_static_blocked(9, 11));
        assert!(!map.is_static_blocked(5, 5));
        assert!(map.is_blocked(9, 9));
        assert!(!map.check_cells_free(&[(9, 9)], None));
        assert_eq!(map.static_rects().len(), 1);
        // Rectangle cells are not explicit static references.
        assert_eq!(map.static_obstacle_keys().count(), 0);
        assert_eq!(map.ref_count(7, 7), 0);
        map.clear_static_rects();
        assert!(!map.is_static_blocked(7, 7));
        assert!(!map.is_static_blocked(9, 9));
    }

    #[test]
    fn set_static_rects_replaces_the_rasterised_bits() {
        let mut map = ObstacleMap::new(8, 8);
        map.add_static_cell(1, 1);
        map.add_static_rect(GridRect {
            x_min: 2,
            y_min: 2,
            x_max: 3,
            y_max: 3,
        });
        map.set_static_rects(&[GridRect {
            x_min: 5,
            y_min: 5,
            x_max: 5,
            y_max: 5,
        }]);
        assert!(!map.is_static_blocked(2, 2));
        assert!(map.is_static_blocked(5, 5));
        assert!(map.is_static_blocked(1, 1), "explicit cells survive");
        map.clear_static_cells();
        assert!(!map.is_static_blocked(1, 1));
        assert!(!map.is_static_blocked(5, 5));
        assert!(map.static_rects().is_empty());
    }

    #[test]
    fn per_cell_accessors_iterate_in_grid_order_with_reference_counts() {
        let mut map = ObstacleMap::new(4, 3);
        map.add_static_cells(&[(3, 2), (0, 0), (2, 1), (2, 1)]);
        let statics: Vec<(i32, i32)> = map.static_obstacle_keys().map(unpack_xy).collect();
        assert_eq!(statics, vec![(0, 0), (2, 1), (3, 2)]);

        assert!(map.commit_route_with_clearance_overlap(1, &[(1, 1)], &[(1, 1), (1, 2)], &[]));
        assert!(map.commit_route_with_clearance_overlap(
            2,
            &[(1, 0)],
            &[(1, 0), (1, 1)],
            &[(1, 1)]
        ));
        let dynamics: Vec<((i32, i32), u16)> = map
            .dynamic_obstacle_entries()
            .map(|(key, refs)| (unpack_xy(key), refs))
            .collect();
        assert_eq!(dynamics, vec![((1, 0), 1), ((1, 1), 2), ((1, 2), 1)]);
        let cores: Vec<((i32, i32), u16)> = map
            .dynamic_core_obstacle_entries()
            .map(|(key, refs)| (unpack_xy(key), refs))
            .collect();
        assert_eq!(cores, vec![((1, 0), 1), ((1, 1), 1)]);
        assert_eq!(map.ref_count(1, 1), 2);

        assert!(map.add_history_cost(3, 0, 4));
        assert!(map.add_congestion_cost(0, 2, 1));
        let history: Vec<((i32, i32), u32)> = map
            .history_entries()
            .map(|(key, cost)| (unpack_xy(key), cost))
            .collect();
        assert_eq!(history, vec![((3, 0), 4)]);
        let congestion: Vec<((i32, i32), u32)> = map
            .congestion_entries()
            .map(|(key, cost)| (unpack_xy(key), cost))
            .collect();
        assert_eq!(congestion, vec![((0, 2), 1)]);
        map.clear_history();
        assert_eq!(map.history_entries().count(), 0);
        assert_eq!(map.congestion_entries().count(), 1);
    }

    #[test]
    fn cloned_map_shares_cold_planes_until_either_side_writes() {
        let mut map = ObstacleMap::new(6, 6);
        assert!(map.commit_route(1, &[(2, 2), (3, 2)]));
        assert!(map.add_static_cell(0, 0));
        assert!(map.add_history_cost(4, 4, 3));

        // The engine's static-only probe: clone, then clear the dynamic part.
        let mut probe = map.clone();
        probe.clear_dynamic();
        assert!(!probe.is_dynamic_blocked(2, 2));
        assert_eq!(probe.ref_count(2, 2), 0);
        assert!(probe.is_static_blocked(0, 0));
        assert_eq!(probe.get_history_cost(4, 4), 3);
        assert!(map.is_dynamic_blocked(2, 2));
        assert_eq!(map.dynamic_owner_at(2, 2), Some(1));

        // Writes on either side stay on that side.
        assert!(probe.add_static_cell(5, 5));
        assert!(!map.is_static_blocked(5, 5));
        assert!(map.remove_static_cell(0, 0));
        assert!(probe.is_static_blocked(0, 0));
        assert!(probe.add_history_cost(4, 4, 1));
        assert_eq!(map.get_history_cost(4, 4), 3);
        assert_eq!(probe.get_history_cost(4, 4), 4);
    }

    #[test]
    fn clear_dynamic_clearance_keeps_core_and_forgets_all_halo_references() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route_with_clearance_overlap(1, &[(2, 2)], &[(2, 2), (2, 3)], &[]));
        assert!(map.commit_route_with_clearance_overlap(
            2,
            &[(4, 4)],
            &[(4, 4), (2, 3)],
            &[(2, 3)]
        ));
        assert_eq!(map.ref_count(2, 3), 2);
        map.clear_dynamic_clearance_in_cells(&[(2, 2), (2, 3), (9, 9)]);
        assert!(map.is_dynamic_blocked(2, 2), "core cells stay blocked");
        assert!(!map.is_dynamic_blocked(2, 3));
        assert_eq!(map.ref_count(2, 3), 0);
        // The owner index is untouched by the clearance opening.
        assert_eq!(map.dynamic_owners_at(2, 3).len(), 2);
        assert!(map.owner_index_is_consistent());
    }

    #[test]
    fn ripup_of_the_first_owner_passes_the_cell_to_a_remaining_owner() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.commit_route_with_clearance_overlap(1, &[(1, 1)], &[(1, 1), (2, 2)], &[]));
        assert!(map.commit_route_with_clearance_overlap(
            2,
            &[(3, 3)],
            &[(3, 3), (2, 2)],
            &[(2, 2)]
        ));
        assert!(map.commit_route_with_clearance_overlap(
            3,
            &[(4, 4)],
            &[(4, 4), (2, 2)],
            &[(2, 2)]
        ));
        assert_eq!(map.dynamic_owner_at(2, 2), Some(1));
        assert!(map.owner_index_is_consistent());
        assert!(map.ripup_route(1));
        let successor = map
            .dynamic_owner_at(2, 2)
            .expect("a remaining owner takes over");
        assert!(successor == 2 || successor == 3);
        assert_eq!(map.dynamic_owners_at(2, 2), [2, 3].into_iter().collect());
        assert!(map.owner_index_is_consistent());
        assert!(map.ripup_route(successor));
        assert_eq!(map.dynamic_owner_at(2, 2), Some(5 - successor));
        assert!(map.owner_index_is_consistent());
        assert!(map.ripup_route(5 - successor));
        assert_eq!(map.dynamic_owner_at(2, 2), None);
        assert!(map.dynamic_owners_at(2, 2).is_empty());
        assert!(map.co_owners.is_empty());
        assert!(map.owner_index_is_consistent());
    }
}
