//! Single-layer obstacle map and routing database primitives.
//!
//! The map is the source of truth for blocked cells. It stores static layout
//! obstacles separately from dynamic routed-net occupancy so future routers can
//! commit, rip up, and reroute nets without rebuilding fixed layout geometry.

use rustc_hash::{FxHashMap, FxHashSet};

/// Packed integer grid-cell key.
pub type CellKey = u64;

/// Application-level net identifier.
pub type NetId = u64;

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

/// Single-layer obstacle map for photonic waveguide routing.
///
/// Static obstacles represent fixed geometry from the Python/gdsfactory side.
/// Dynamic obstacles represent committed routed waveguides and can be ripped up
/// by net. Both maps use reference counts, so overlapping blockages remain
/// blocked until every owner is removed.
#[derive(Clone, Debug)]
pub struct ObstacleMap {
    width: i32,
    height: i32,
    static_obstacles: FxHashMap<CellKey, u16>,
    dynamic_obstacles: FxHashMap<CellKey, u16>,
    net_routes: FxHashMap<NetId, Vec<CellKey>>,
    history_cost: FxHashMap<CellKey, u32>,
}

impl ObstacleMap {
    /// Create an empty single-layer obstacle map with dimensions in grid cells.
    ///
    /// Valid coordinates are `0 <= x < width` and `0 <= y < height`.
    pub fn new(width: i32, height: i32) -> Self {
        assert!(width >= 0, "width must be non-negative");
        assert!(height >= 0, "height must be non-negative");

        Self {
            width,
            height,
            static_obstacles: FxHashMap::default(),
            dynamic_obstacles: FxHashMap::default(),
            net_routes: FxHashMap::default(),
            history_cost: FxHashMap::default(),
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

    /// Add one static obstacle reference. Returns false if the cell is out of bounds.
    pub fn add_static_cell(&mut self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return false;
        }
        Self::increment_ref(&mut self.static_obstacles, pack_xy(x, y));
        true
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

    /// Remove one static obstacle reference. Returns true when a reference was removed.
    pub fn remove_static_cell(&mut self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return false;
        }
        Self::decrement_ref(&mut self.static_obstacles, pack_xy(x, y))
    }

    /// Return true if a static obstacle blocks this in-bounds cell.
    ///
    /// Out-of-bounds cells are not static obstacles, but they are considered
    /// unavailable by [`Self::is_blocked`] and free-space checks.
    pub fn is_static_blocked(&self, x: i32, y: i32) -> bool {
        self.in_bounds(x, y) && self.static_obstacles.contains_key(&pack_xy(x, y))
    }

    /// Return true if a committed route blocks this in-bounds cell.
    pub fn is_dynamic_blocked(&self, x: i32, y: i32) -> bool {
        self.in_bounds(x, y) && self.dynamic_obstacles.contains_key(&pack_xy(x, y))
    }

    /// Return true if the cell is unavailable for routing.
    ///
    /// Out-of-bounds cells are treated as blocked.
    pub fn is_blocked(&self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return true;
        }
        let key = pack_xy(x, y);
        self.static_obstacles.contains_key(&key) || self.dynamic_obstacles.contains_key(&key)
    }

    /// Total static plus dynamic reference count for an in-bounds cell.
    pub fn ref_count(&self, x: i32, y: i32) -> u32 {
        if !self.in_bounds(x, y) {
            return 0;
        }
        let key = pack_xy(x, y);
        self.static_obstacles.get(&key).copied().unwrap_or(0) as u32
            + self.dynamic_obstacles.get(&key).copied().unwrap_or(0) as u32
    }

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

        self.ripup_route(net_id);

        for &key in &keys {
            Self::increment_ref(&mut self.dynamic_obstacles, key);
        }
        self.net_routes.insert(net_id, keys);
        true
    }

    /// Rip up a previously committed route. Returns true when a route existed.
    pub fn ripup_route(&mut self, net_id: NetId) -> bool {
        let Some(keys) = self.net_routes.remove(&net_id) else {
            return false;
        };

        for key in keys {
            Self::decrement_ref(&mut self.dynamic_obstacles, key);
        }
        true
    }

    /// Clear every dynamic route and routed-net owner entry.
    pub fn clear_dynamic(&mut self) {
        self.dynamic_obstacles.clear();
        self.net_routes.clear();
    }

    /// Return the packed cells owned by `net_id`, if that net has a committed route.
    pub fn get_net_cells(&self, net_id: NetId) -> Option<&[CellKey]> {
        self.net_routes.get(&net_id).map(Vec::as_slice)
    }

    /// Add a history/congestion penalty to an in-bounds cell.
    ///
    /// Returns false if the cell is out of bounds. Cost addition saturates at
    /// `u32::MAX`.
    pub fn add_history_cost(&mut self, x: i32, y: i32, amount: u32) -> bool {
        if !self.in_bounds(x, y) {
            return false;
        }

        let entry = self.history_cost.entry(pack_xy(x, y)).or_insert(0);
        *entry = entry.saturating_add(amount);
        true
    }

    /// Return the history/congestion cost for a cell, or zero if none exists.
    pub fn get_history_cost(&self, x: i32, y: i32) -> u32 {
        if !self.in_bounds(x, y) {
            return 0;
        }
        self.history_cost.get(&pack_xy(x, y)).copied().unwrap_or(0)
    }

    /// Clear all history/congestion penalties.
    pub fn clear_history(&mut self) {
        self.history_cost.clear();
    }

    /// Check whether every listed cell is in bounds and free.
    ///
    /// `opened_cells` temporarily overrides static and dynamic blocking for
    /// source/target port cells. It does not make out-of-bounds cells valid.
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

        let mut inflated = FxHashSet::default();

        for &(x, y) in cells {
            if !self.in_bounds(x, y) {
                continue;
            }

            for dx in -clearance_radius..=clearance_radius {
                for dy in -clearance_radius..=clearance_radius {
                    let inside_metric = match metric {
                        ClearanceMetric::Manhattan => dx.abs() + dy.abs() <= clearance_radius,
                        ClearanceMetric::Chebyshev => dx.abs().max(dy.abs()) <= clearance_radius,
                    };

                    if !inside_metric {
                        continue;
                    }

                    let nx = x + dx;
                    let ny = y + dy;
                    if self.in_bounds(nx, ny) {
                        inflated.insert(pack_xy(nx, ny));
                    }
                }
            }
        }

        inflated
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

    #[inline]
    fn check_cell_free(&self, x: i32, y: i32, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        if !self.in_bounds(x, y) {
            return false;
        }

        let key = pack_xy(x, y);
        if let Some(opened) = opened_cells {
            if opened.contains(&key) {
                return true;
            }
        }

        !self.static_obstacles.contains_key(&key) && !self.dynamic_obstacles.contains_key(&key)
    }

    fn increment_ref(map: &mut FxHashMap<CellKey, u16>, key: CellKey) {
        let count = map.entry(key).or_insert(0);
        *count = count
            .checked_add(1)
            .expect("cell reference count overflowed u16");
    }

    fn decrement_ref(map: &mut FxHashMap<CellKey, u16>, key: CellKey) -> bool {
        let Some(count) = map.get_mut(&key) else {
            return false;
        };

        if *count > 1 {
            *count -= 1;
        } else {
            map.remove(&key);
        }

        true
    }
}
