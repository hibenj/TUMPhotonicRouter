//! The meander stage: analytic comb geometry, obstacle-aware search and
//! the registered path-length-matching planner (phase 6 slice C of the
//! 2026-10 do-over gathered the three former top-level files here).
//!
//! * `geometry` -- the analytic physical meander planner: a comb of
//!   U-turns in a box, its exact inserted length and footprint.
//! * `search` -- the obstacle-aware layer over it: straight-run
//!   extraction from a centerline, the dense occupancy prefix and sparse
//!   cell indices that answer "is this box free", and the depth sweep that
//!   picks a run, a side and a box depth.
//! * `plan` -- the registered planner the Python path-length-matching
//!   stage drives: routes and their geometries registered once, then the
//!   requirement requests (candidate bundles, the same-edge split) and the
//!   final sequence planned against the shared reserved cells.
//!
//! Every public item is re-exported here, so `crate::meander::X` names
//! any of them; the bindings and `geometry_realization` reach the
//! crate-private helpers through the submodule paths.

pub mod geometry;
pub mod plan;
pub mod search;

pub use geometry::*;
pub use plan::*;
pub use search::*;
