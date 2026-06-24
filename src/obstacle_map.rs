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

/// Inclusive integer-grid rectangle for compact obstacle storage.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct GridRect {
    pub x_min: i32,
    pub y_min: i32,
    pub x_max: i32,
    pub y_max: i32,
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
    occupancy: Vec<u8>,
    static_rects: Vec<GridRect>,
    static_obstacles: FxHashMap<CellKey, u16>,
    dynamic_obstacles: FxHashMap<CellKey, u16>,
    dynamic_cell_owner: FxHashMap<CellKey, NetId>,
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
            occupancy: vec![0; cell_count],
            static_rects: Vec::new(),
            static_obstacles: FxHashMap::default(),
            dynamic_obstacles: FxHashMap::default(),
            dynamic_cell_owner: FxHashMap::default(),
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
        let key = pack_xy(x, y);
        if Self::increment_ref(&mut self.static_obstacles, key) {
            self.set_occupancy_bit(x, y, STATIC_BIT);
        }
        true
    }

    /// Add a compact static rectangle.
    pub fn add_static_rect(&mut self, rect: GridRect) {
        if let Some(rect) = normalize_rect(rect) {
            self.static_rects.push(rect);
        }
    }

    /// Add compact static rectangles.
    pub fn add_static_rects(&mut self, rects: &[GridRect]) {
        self.static_rects
            .extend(rects.iter().copied().filter_map(normalize_rect));
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
            if !self.in_bounds(x, y) {
                continue;
            }
            if Self::increment_ref(&mut self.static_obstacles, key) {
                self.set_occupancy_bit(x, y, STATIC_BIT);
            }
            added += 1;
        }
        added
    }

    /// Remove all compact static rectangles.
    pub fn clear_static_rects(&mut self) {
        self.static_rects.clear();
    }

    /// Replace all compact static rectangles.
    pub fn set_static_rects(&mut self, rects: &[GridRect]) {
        self.static_rects.clear();
        self.add_static_rects(rects);
    }

    /// Compact static rectangles currently used by the map.
    pub fn static_rects(&self) -> &[GridRect] {
        &self.static_rects
    }

    /// Packed cell keys with explicit static obstacle references.
    pub fn static_obstacle_keys(&self) -> impl Iterator<Item = CellKey> + '_ {
        self.static_obstacles.keys().copied()
    }

    /// Packed cell keys with dynamic route obstacle references.
    pub fn dynamic_obstacle_keys(&self) -> impl Iterator<Item = CellKey> + '_ {
        self.dynamic_obstacles.keys().copied()
    }

    /// Packed cell keys with accumulated rip-up history costs.
    pub fn history_entries(&self) -> impl Iterator<Item = (CellKey, u32)> + '_ {
        self.history_cost.iter().map(|(&key, &cost)| (key, cost))
    }

    /// Remove every static obstacle entry (compact + cell-based).
    pub fn clear_static_cells(&mut self) {
        self.static_obstacles.clear();
        self.static_rects.clear();
        for cell in &mut self.occupancy {
            *cell &= !STATIC_BIT;
        }
    }

    /// Remove one static obstacle reference. Returns true when a reference was removed.
    pub fn remove_static_cell(&mut self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return false;
        }
        let key = pack_xy(x, y);
        let removed = Self::decrement_ref(&mut self.static_obstacles, key);
        if removed && !self.static_obstacles.contains_key(&key) {
            self.clear_occupancy_bit(x, y, STATIC_BIT);
        }
        removed
    }

    /// Return true if a static obstacle blocks this in-bounds cell.
    ///
    /// Out-of-bounds cells are not static obstacles, but they are considered
    /// unavailable by [`Self::is_blocked`] and free-space checks.
    pub fn is_static_blocked(&self, x: i32, y: i32) -> bool {
        self.read_occupancy_bit(x, y, STATIC_BIT) || self.is_static_rect_blocked(x, y)
    }

    /// Return true if a committed route blocks this in-bounds cell.
    pub fn is_dynamic_blocked(&self, x: i32, y: i32) -> bool {
        self.read_occupancy_bit(x, y, DYNAMIC_BIT)
    }

    /// Return true if the cell is unavailable for routing.
    ///
    /// Out-of-bounds cells are treated as blocked.
    pub fn is_blocked(&self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return true;
        }
        self.read_occupancy_bit(x, y, STATIC_BIT)
            || self.read_occupancy_bit(x, y, DYNAMIC_BIT)
            || self.is_static_rect_blocked(x, y)
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

        let old_route_keys: FxHashSet<CellKey> = self
            .net_routes
            .get(&net_id)
            .map(|route| route.iter().copied().collect())
            .unwrap_or_default();
        for &key in &keys {
            let existing_refs = self.dynamic_obstacles.get(&key).copied().unwrap_or(0);
            let same_net_refs = u16::from(old_route_keys.contains(&key));
            if existing_refs > same_net_refs {
                return false;
            }
        }

        self.ripup_route(net_id);

        for &key in &keys {
            let (x, y) = unpack_xy(key);
            if Self::increment_ref(&mut self.dynamic_obstacles, key) {
                self.set_occupancy_bit(x, y, DYNAMIC_BIT);
            }
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
            let (x, y) = unpack_xy(key);
            let removed = Self::decrement_ref(&mut self.dynamic_obstacles, key);
            if removed && !self.dynamic_obstacles.contains_key(&key) {
                self.clear_occupancy_bit(x, y, DYNAMIC_BIT);
            }
        }
        true
    }

    /// Clear every dynamic route and routed-net owner entry.
    pub fn clear_dynamic(&mut self) {
        self.dynamic_obstacles.clear();
        self.dynamic_cell_owner.clear();
        self.net_routes.clear();
        for cell in &mut self.occupancy {
            *cell &= !DYNAMIC_BIT;
        }
    }

    /// Return the packed cells owned by `net_id`, if that net has a committed route.
    pub fn get_net_cells(&self, net_id: NetId) -> Option<&[CellKey]> {
        self.net_routes.get(&net_id).map(Vec::as_slice)
    }

    /// Return the committed dynamic-route owner of a cell, if any.
    pub fn dynamic_owner_at(&self, x: i32, y: i32) -> Option<NetId> {
        if !self.in_bounds(x, y) {
            return None;
        }
        let key = pack_xy(x, y);
        if let Some(owner) = self.dynamic_cell_owner.get(&key) {
            return Some(*owner);
        }
        self.net_routes
            .iter()
            .find_map(|(&net_id, cells)| cells.contains(&key).then_some(net_id))
    }

    /// Return all dynamic-route owners intersecting the provided cells.
    pub fn dynamic_owners_for_cells(&self, cells: &[(i32, i32)]) -> FxHashSet<NetId> {
        let query: FxHashSet<CellKey> = cells
            .iter()
            .filter_map(|&(x, y)| self.in_bounds(x, y).then_some(pack_xy(x, y)))
            .collect();
        let mut owners = FxHashSet::default();
        if query.is_empty() {
            return owners;
        }
        for (&net_id, route_cells) in &self.net_routes {
            if route_cells.iter().any(|cell| query.contains(cell)) {
                owners.insert(net_id);
            }
        }
        owners
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
            occupancy: self.occupancy.clone(),
            static_rects: self.static_rects.clone(),
            static_obstacles: self.static_obstacles.clone(),
            dynamic_obstacles: self.dynamic_obstacles.clone(),
            // Expanded search maps only need dynamic occupancy, not ownership
            // or per-net route records. Keeping these empty avoids copying and
            // updating large owner maps on every normal route.
            dynamic_cell_owner: FxHashMap::default(),
            net_routes: FxHashMap::default(),
            history_cost: self.history_cost.clone(),
        };
        let source_keys: Vec<CellKey> = self.dynamic_obstacles.keys().copied().collect();
        for key in source_keys {
            let (x, y) = unpack_xy(key);
            for dx in -radius_cells..=radius_cells {
                for dy in -radius_cells..=radius_cells {
                    let nx = x + dx;
                    let ny = y + dy;
                    if !expanded.in_bounds(nx, ny) {
                        continue;
                    }
                    let expanded_key = pack_xy(nx, ny);
                    if Self::increment_ref(&mut expanded.dynamic_obstacles, expanded_key) {
                        expanded.set_occupancy_bit(nx, ny, DYNAMIC_BIT);
                    }
                }
            }
        }
        expanded
    }

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
        if self.rect_blocked_by_static_rects(&rect, opened_cells) {
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

    /// Return true when there is a compact-rect static blockage in `rect`.
    fn rect_blocked_by_static_rects(
        &self,
        query: &GridRect,
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        for static_rect in &self.static_rects {
            let Some(overlap) = intersect_rects(query, static_rect) else {
                continue;
            };
            let Some(opened_cells) = opened_cells else {
                return true;
            };
            if grid_rect_area_cells(overlap) > opened_cells.len() {
                return true;
            }
            for y in overlap.y_min..=overlap.y_max {
                for x in overlap.x_min..=overlap.x_max {
                    if !opened_cells.contains(&pack_xy(x, y)) {
                        return true;
                    }
                }
            }
        }
        false
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
        if self.read_occupancy_bit(x, y, DYNAMIC_BIT) {
            return false;
        }
        if let Some(opened) = opened_cells {
            if opened.contains(&key) {
                return true;
            }
        }

        !self.is_static_blocked(x, y)
    }

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

    #[inline]
    fn set_occupancy_bit(&mut self, x: i32, y: i32, bit: u8) {
        if let Some(idx) = self.dense_idx(x, y) {
            self.occupancy[idx] |= bit;
        }
    }

    #[inline]
    fn clear_occupancy_bit(&mut self, x: i32, y: i32, bit: u8) {
        if let Some(idx) = self.dense_idx(x, y) {
            self.occupancy[idx] &= !bit;
        }
    }

    fn is_static_rect_blocked(&self, x: i32, y: i32) -> bool {
        self.static_rects
            .iter()
            .any(|rect| x >= rect.x_min && x <= rect.x_max && y >= rect.y_min && y <= rect.y_max)
    }

    #[inline]
    fn read_occupancy_bit(&self, x: i32, y: i32, bit: u8) -> bool {
        self.dense_idx(x, y)
            .and_then(|idx| self.occupancy.get(idx).copied())
            .map(|value| (value & bit) != 0)
            .unwrap_or(false)
    }

    fn increment_ref(map: &mut FxHashMap<CellKey, u16>, key: CellKey) -> bool {
        let count = map.entry(key).or_insert(0);
        let was_zero = *count == 0;
        *count = count
            .checked_add(1)
            .expect("cell reference count overflowed u16");
        was_zero
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

#[inline]
fn intersect_rects(a: &GridRect, b: &GridRect) -> Option<GridRect> {
    let x_min = a.x_min.max(b.x_min);
    let y_min = a.y_min.max(b.y_min);
    let x_max = a.x_max.min(b.x_max);
    let y_max = a.y_max.min(b.y_max);

    if x_min > x_max || y_min > y_max {
        None
    } else {
        Some(GridRect {
            x_min,
            y_min,
            x_max,
            y_max,
        })
    }
}

#[inline]
fn grid_rect_area_cells(rect: GridRect) -> usize {
    let width = i64::from(rect.x_max - rect.x_min + 1);
    let height = i64::from(rect.y_max - rect.y_min + 1);
    usize::try_from(width.saturating_mul(height)).unwrap_or(usize::MAX)
}

fn normalize_rect(rect: GridRect) -> Option<GridRect> {
    if rect.x_min > rect.x_max || rect.y_min > rect.y_max {
        return None;
    }
    if rect.x_min < 0 || rect.y_min < 0 {
        return None;
    }
    // The map-level bounds are validated by callers (`rect_blocked`,
    // `check_cell_free`, and `check_cell` methods) before use.
    Some(rect)
}

const STATIC_BIT: u8 = 1 << 0;
const DYNAMIC_BIT: u8 = 1 << 1;

#[cfg(test)]
mod tests {
    use super::*;

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

        let owners = map.dynamic_owners_for_cells(&[(1, 1), (2, 2), (3, 2)]);
        assert_eq!(owners.len(), 1);
        assert!(owners.contains(&11));

        assert!(map.commit_route(11, &[(4, 2)]));
        assert_eq!(map.dynamic_owner_at(2, 2), None);
        assert_eq!(map.dynamic_owner_at(4, 2), Some(11));
        assert!(map.ripup_route(11));
        assert_eq!(map.dynamic_owner_at(4, 2), None);
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
}
