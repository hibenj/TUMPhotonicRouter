//! SVG export of a routed net over its obstacle map, for debugging.
//! Moved out of `src/astar.rs` (Milestone 3, Slice 2); pure code motion.

use crate::obstacle_map::{unpack_xy, CellKey, ObstacleMap};
use crate::search::state::RouteResult;
use rustc_hash::FxHashSet;

/// Export an SVG string showing obstacles and a routed path.
pub fn export_route_svg(obstacle_map: &ObstacleMap, route_result: &RouteResult) -> String {
    export_route_svg_with_port_open_cells(obstacle_map, route_result, None)
}

pub fn export_route_svg_with_port_open_cells(
    obstacle_map: &ObstacleMap,
    route_result: &RouteResult,
    port_open_cells: Option<&FxHashSet<CellKey>>,
) -> String {
    let width = obstacle_map.width();
    let height = obstacle_map.height();
    if width <= 0 || height <= 0 {
        return r#"<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1" />"#.to_string();
    }

    let cell_px = (1200 / width.max(height).max(1)).clamp(1, 8);
    let width_px = width * cell_px;
    let height_px = height * cell_px;
    let mut svg = String::new();

    svg.push_str(&format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" viewBox="0 0 {width} {height}">"#
    ));
    svg.push_str(r##"<rect width="100%" height="100%" fill="#eeeeee" />"##);
    svg.push_str(r#"<path d=""#);
    for x in 0..=width {
        svg.push_str(&format!("M {x} 0 V {height} "));
    }
    for y in 0..=height {
        svg.push_str(&format!("M 0 {y} H {width} "));
    }
    svg.push_str(
        r##"" stroke="#f2f2f2" stroke-width="0.025" vector-effect="non-scaling-stroke" opacity="0.65" fill="none" />"##,
    );

    for x in 0..width {
        for y in 0..height {
            if obstacle_map.is_static_blocked(x, y) {
                let svg_y = height - y - 1;
                svg.push_str(&format!(
                    r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#d9d9d9" opacity="0.75" />"##
                ));
            }
        }
    }

    for x in 0..width {
        for y in 0..height {
            if obstacle_map.is_dynamic_blocked(x, y) {
                let svg_y = height - y - 1;
                svg.push_str(&format!(
                    r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#000000" opacity="0.92" />"##
                ));
            }
        }
    }

    if let Some(port_open_cells) = port_open_cells {
        let mut cells: Vec<CellKey> = port_open_cells.iter().copied().collect();
        cells.sort_unstable();
        for key in cells {
            let (x, y) = unpack_xy(key);
            if obstacle_map.in_bounds(x, y) {
                let svg_y = height - y - 1;
                svg.push_str(&format!(
                    r##"<rect class="port-access" x="{x}" y="{svg_y}" width="1" height="1" fill="#d93025" opacity="0.38" />"##
                ));
            }
        }
    }

    for &(x, y) in &route_result.cells {
        if obstacle_map.in_bounds(x, y) {
            let svg_y = height - y - 1;
            svg.push_str(&format!(
                r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#1a73e8" />"##
            ));
        }
    }

    svg.push_str("</svg>\n");
    svg
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::search::astar::route_single_net;
    use crate::search::state::State;
    use crate::search::test_support::*;

    #[test]
    fn exports_route_svg() {
        let map = ObstacleMap::new(6, 3);
        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
        )
        .expect("route should exist");

        let svg = export_route_svg(&map, &result);
        assert!(svg.contains("<svg"));
        assert!(svg.contains("#1a73e8"));
    }
}
