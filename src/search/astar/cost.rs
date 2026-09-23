//! Per-primitive search-cost metadata. Slice 3 of Milestone 3 adds the cost
//! functions extracted from the kernel loop; today this module only holds
//! the metadata type they will operate on. Moved out of `src/astar.rs`
//! (Milestone 3, Slice 2); pure code motion, no behaviour change.

use crate::primitives::Primitive;
use crate::search::astar::expansion::primitive_transition_class;

#[derive(Clone, Copy, Debug)]
pub(crate) struct PrimitiveSearchMetadata {
    pub(crate) transition_class: usize,
    pub(crate) base_step_cost: f64,
}

impl PrimitiveSearchMetadata {
    pub(crate) fn from_primitive(primitive: &Primitive, bend_weight: f64) -> Self {
        Self {
            transition_class: primitive_transition_class(
                &primitive.geometry,
                primitive.dx,
                primitive.dy,
            ),
            base_step_cost: primitive.length_um + bend_weight * primitive.bend_cost,
        }
    }
}
