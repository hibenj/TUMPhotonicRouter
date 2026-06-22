//! Photonic waveguide movement primitives.
//!
//! The first router uses discrete approximations of photonic moves. Each
//! primitive carries both a state transition and the grid footprint that must be
//! collision-free before the move is accepted.

/// Physical geometry intent used for post-routing realization.
///
/// This is separate from [`Primitive::footprint`], which is the conservative
/// discrete representation used for A* collision checks.
#[derive(Clone, Debug, PartialEq)]
pub enum PrimitiveGeometry {
    Straight { length_um: f64 },
    Bend { radius_um: f64, angle_delta: i8 },
}

/// Eight grid headings in 45-degree increments.
///
/// Angle `0` is east, then angles increase counter-clockwise:
/// `1 = northeast`, `2 = north`, ... `7 = southeast`.
pub const DIRECTIONS: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

/// A precomputed photonic movement primitive.
#[derive(Clone, Debug, PartialEq)]
pub struct Primitive {
    pub id: u16,
    pub start_angle: u8,
    pub end_angle: u8,
    pub dx: i32,
    pub dy: i32,
    /// Conservative discrete occupancy used for search and collision checks.
    pub footprint: Vec<(i32, i32)>,
    pub length_um: f64,
    pub bend_cost: f64,
    /// Physical realization descriptor used after route selection.
    pub geometry: PrimitiveGeometry,
}

/// Configuration used to create the first photonic primitive library.
#[derive(Clone, Debug)]
pub struct PrimitiveLibraryConfig {
    pub grid_size_um: f64,
    pub straight_short_cells: i32,
    pub straight_long_cells: i32,
    pub bend_radius_cells: i32,
    pub allow_45_degree_turns: bool,
}

impl Default for PrimitiveLibraryConfig {
    fn default() -> Self {
        Self {
            grid_size_um: 0.5,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 2,
            allow_45_degree_turns: true,
        }
    }
}

/// Primitive lookup table indexed by start angle.
#[derive(Clone, Debug)]
pub struct PrimitiveLibrary {
    primitives_per_angle: Vec<Vec<Primitive>>,
    grid_size_um: f64,
}

impl PrimitiveLibrary {
    pub fn new(primitives_per_angle: Vec<Vec<Primitive>>, grid_size_um: f64) -> Self {
        assert_eq!(
            primitives_per_angle.len(),
            8,
            "primitive library must contain exactly 8 angle buckets"
        );
        Self {
            primitives_per_angle,
            grid_size_um,
        }
    }

    /// Return all primitives valid for a start angle.
    pub fn get_primitives_for_angle(&self, angle: u8) -> &[Primitive] {
        &self.primitives_per_angle[(angle % 8) as usize]
    }

    /// Grid resolution used when converting geometric distance to micrometers.
    pub fn grid_size_um(&self) -> f64 {
        self.grid_size_um
    }
}

/// Create the initial photonic primitive library.
///
/// The starting set contains:
/// - short straight
/// - long straight
/// - 45-degree left/right bend when enabled
/// - 90-degree left/right bend
pub fn create_photonic_primitive_library(config: PrimitiveLibraryConfig) -> PrimitiveLibrary {
    assert!(config.grid_size_um > 0.0, "grid_size_um must be positive");
    assert!(
        config.straight_short_cells > 0,
        "straight_short_cells must be positive"
    );
    assert!(
        config.straight_long_cells > 0,
        "straight_long_cells must be positive"
    );
    assert!(
        config.bend_radius_cells > 0,
        "bend_radius_cells must be positive"
    );

    let mut next_id = 0u16;
    let mut primitives_per_angle = Vec::with_capacity(8);

    for angle in 0..8u8 {
        let mut primitives = Vec::with_capacity(if config.allow_45_degree_turns { 6 } else { 4 });

        primitives.push(make_straight(
            next_primitive_id(&mut next_id),
            angle,
            config.straight_short_cells,
            config.grid_size_um,
        ));
        primitives.push(make_straight(
            next_primitive_id(&mut next_id),
            angle,
            config.straight_long_cells,
            config.grid_size_um,
        ));
        if config.allow_45_degree_turns {
            primitives.push(make_turn(
                next_primitive_id(&mut next_id),
                angle,
                1,
                config.bend_radius_cells,
                config.grid_size_um,
            ));
            primitives.push(make_turn(
                next_primitive_id(&mut next_id),
                angle,
                -1,
                config.bend_radius_cells,
                config.grid_size_um,
            ));
        }
        primitives.push(make_turn(
            next_primitive_id(&mut next_id),
            angle,
            2,
            config.bend_radius_cells,
            config.grid_size_um,
        ));
        primitives.push(make_turn(
            next_primitive_id(&mut next_id),
            angle,
            -2,
            config.bend_radius_cells,
            config.grid_size_um,
        ));

        primitives_per_angle.push(primitives);
    }

    PrimitiveLibrary::new(primitives_per_angle, config.grid_size_um)
}

/// Create a plain 4-connected unit-step grid library for accelerator experiments.
///
/// This is intentionally not photonic: it has no bend primitives, no diagonal
/// states, and one unit straight move in each cardinal direction. It is used to
/// benchmark grid-search accelerators against a matching baseline without
/// weakening the photonic primitive eligibility guard.
pub fn create_jps4_unit_grid_primitive_library(grid_size_um: f64) -> PrimitiveLibrary {
    assert!(grid_size_um > 0.0, "grid_size_um must be positive");
    let mut next_id = 0u16;
    let mut primitives_per_angle = Vec::with_capacity(8);
    for angle in 0..8u8 {
        if angle % 2 == 0 {
            primitives_per_angle.push(vec![make_straight(
                next_primitive_id(&mut next_id),
                angle,
                1,
                grid_size_um,
            )]);
        } else {
            primitives_per_angle.push(Vec::new());
        }
    }
    PrimitiveLibrary::new(primitives_per_angle, grid_size_um)
}

/// Create a plain 4-connected unit-step grid library for baseline A* experiments.
///
/// Every state can step in any cardinal direction with uniform cost. This
/// matches the topology used by the JPS4 prototype, but it intentionally fails
/// JPS4 eligibility so it remains a baseline A* mode.
pub fn create_grid4_unit_grid_primitive_library(grid_size_um: f64) -> PrimitiveLibrary {
    assert!(grid_size_um > 0.0, "grid_size_um must be positive");
    let mut next_id = 0u16;
    let mut primitives_per_angle = Vec::with_capacity(8);
    for start_angle in 0..8u8 {
        let mut primitives = Vec::with_capacity(4);
        for end_angle in [0u8, 2, 4, 6] {
            let dir = direction(end_angle);
            primitives.push(Primitive {
                id: next_primitive_id(&mut next_id),
                start_angle,
                end_angle,
                dx: dir.0,
                dy: dir.1,
                footprint: vec![(0, 0), dir],
                length_um: grid_size_um,
                bend_cost: 0.0,
                geometry: PrimitiveGeometry::Straight {
                    length_um: grid_size_um,
                },
            });
        }
        primitives_per_angle.push(primitives);
    }
    PrimitiveLibrary::new(primitives_per_angle, grid_size_um)
}

fn make_straight(id: u16, angle: u8, cells: i32, grid_size_um: f64) -> Primitive {
    let dir = direction(angle);
    let footprint = line_footprint((0, 0), dir, cells);
    let dx = dir.0 * cells;
    let dy = dir.1 * cells;

    let length_um = vector_length_um(dx, dy, grid_size_um);
    Primitive {
        id,
        start_angle: angle,
        end_angle: angle,
        dx,
        dy,
        footprint,
        length_um,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um },
    }
}

fn make_turn(
    id: u16,
    start_angle: u8,
    angle_delta: i8,
    radius_cells: i32,
    grid_size_um: f64,
) -> Primitive {
    let end_angle = wrap_angle(start_angle as i8 + angle_delta);
    let start_dir = direction(start_angle);
    let end_dir = direction(end_angle);

    let mut footprint = line_footprint((0, 0), start_dir, radius_cells);
    let corner = (start_dir.0 * radius_cells, start_dir.1 * radius_cells);
    let second_leg = line_footprint(corner, end_dir, radius_cells);
    extend_unique(&mut footprint, second_leg);

    let end = footprint
        .last()
        .copied()
        .expect("turn primitive footprint cannot be empty");

    Primitive {
        id,
        start_angle,
        end_angle,
        dx: end.0,
        dy: end.1,
        footprint,
        length_um: vector_length_um(
            start_dir.0 * radius_cells,
            start_dir.1 * radius_cells,
            grid_size_um,
        ) + vector_length_um(
            end_dir.0 * radius_cells,
            end_dir.1 * radius_cells,
            grid_size_um,
        ),
        bend_cost: angle_delta.unsigned_abs() as f64,
        geometry: PrimitiveGeometry::Bend {
            radius_um: radius_cells as f64 * grid_size_um,
            angle_delta,
        },
    }
}

fn line_footprint(start: (i32, i32), dir: (i32, i32), cells: i32) -> Vec<(i32, i32)> {
    let mut points = Vec::with_capacity((cells + 1) as usize);
    for step in 0..=cells {
        points.push((start.0 + dir.0 * step, start.1 + dir.1 * step));
    }
    points
}

fn extend_unique(base: &mut Vec<(i32, i32)>, extra: Vec<(i32, i32)>) {
    for point in extra {
        if base.last().copied() != Some(point) {
            base.push(point);
        }
    }
}

fn vector_length_um(dx: i32, dy: i32, grid_size_um: f64) -> f64 {
    ((dx * dx + dy * dy) as f64).sqrt() * grid_size_um
}

fn direction(angle: u8) -> (i32, i32) {
    DIRECTIONS[(angle % 8) as usize]
}

fn wrap_angle(angle: i8) -> u8 {
    angle.rem_euclid(8) as u8
}

fn next_primitive_id(next_id: &mut u16) -> u16 {
    let id = *next_id;
    *next_id = next_id.checked_add(1).expect("primitive id overflowed u16");
    id
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn creates_primitives_for_each_angle() {
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig::default());

        for angle in 0..8 {
            let primitives = library.get_primitives_for_angle(angle);
            assert_eq!(primitives.len(), 6);
            assert!(primitives
                .iter()
                .all(|primitive| primitive.start_angle == angle));
        }
    }

    #[test]
    fn can_disable_forty_five_degree_turns() {
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig {
            allow_45_degree_turns: false,
            ..PrimitiveLibraryConfig::default()
        });

        for angle in 0..8 {
            let primitives = library.get_primitives_for_angle(angle);
            assert_eq!(primitives.len(), 4);
            assert!(primitives.iter().all(|primitive| {
                let delta =
                    (primitive.end_angle as i16 - primitive.start_angle as i16).rem_euclid(8);
                delta == 0 || delta == 2 || delta == 6
            }));
        }
    }

    #[test]
    fn east_straight_short_moves_one_cell() {
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig::default());
        let primitive = &library.get_primitives_for_angle(0)[0];

        assert_eq!(primitive.dx, 1);
        assert_eq!(primitive.dy, 0);
        assert_eq!(primitive.end_angle, 0);
        assert_eq!(primitive.footprint, vec![(0, 0), (1, 0)]);
        assert_eq!(
            primitive.geometry,
            PrimitiveGeometry::Straight {
                length_um: primitive.length_um
            }
        );
    }

    #[test]
    fn east_ninety_left_turn_ends_north() {
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig::default());
        let primitive = library
            .get_primitives_for_angle(0)
            .iter()
            .find(|primitive| primitive.end_angle == 2)
            .expect("east-to-north 90-degree turn should exist");

        assert_eq!(primitive.end_angle, 2);
        assert_eq!(primitive.dx, 2);
        assert_eq!(primitive.dy, 2);
        assert!(primitive.bend_cost > 0.0);
        assert!(primitive.footprint.contains(&(2, 2)));
        assert!(matches!(
            primitive.geometry,
            PrimitiveGeometry::Bend {
                radius_um: _,
                angle_delta: 2
            }
        ));
    }

    #[test]
    fn every_primitive_has_geometry() {
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig::default());
        for angle in 0..8 {
            for primitive in library.get_primitives_for_angle(angle) {
                match primitive.geometry {
                    PrimitiveGeometry::Straight { length_um } => assert!(length_um > 0.0),
                    PrimitiveGeometry::Bend {
                        radius_um,
                        angle_delta,
                    } => {
                        assert!(radius_um > 0.0);
                        assert!(angle_delta != 0);
                    }
                }
            }
        }
    }
}
