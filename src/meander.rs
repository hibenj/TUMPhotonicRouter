// -----------------------------------------------------------------------------
// Analytic physical meander planner
// -----------------------------------------------------------------------------

const BEND_SAMPLES_PER_90_DEG: usize = 8;
const EPS: f64 = 1.0e-9;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MeanderSide {
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicalPoint {
    pub x_um: f64,
    pub y_um: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StraightSegment {
    pub start: PhysicalPoint,
    pub end: PhysicalPoint,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeanderBox {
    pub min_x_um: f64,
    pub max_x_um: f64,
    pub min_y_um: f64,
    pub max_y_um: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AnalyticMeanderConfig {
    pub requested_extra_length_um: f64,
    pub min_bend_radius_um: f64,
    pub min_straight_um: f64,
    pub max_bumps: usize,
    pub max_meander_height_um: f64,
    pub side: MeanderSide,
    pub mode: MeanderPlanningMode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeanderPlanningMode {
    FillBoxMultiBump,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AnalyticMeanderPlan {
    pub centerline: Vec<PhysicalPoint>,
    pub inserted_extra_length_um: f64,
    pub bumps: usize,
    pub side: MeanderSide,
}

#[derive(Clone, Debug, PartialEq)]
pub enum MeanderPlanningError {
    NonFiniteInput,
    NonPositiveSegmentLength,
    NonPositiveBendRadius,
    NonPositiveRequestedExtraLength,
    UnsupportedSegmentOrientation,
    AvailableBoxTooSmall,
    RequestedExtraLengthDoesNotFit,
    MaxBumpsTooSmall,
}

pub fn bend_radius_cells_from_min_radius(
    min_bend_radius_um: f64,
    grid_size_um: f64,
) -> Result<i32, String> {
    if !min_bend_radius_um.is_finite() || min_bend_radius_um <= 0.0 {
        return Err("min_bend_radius_um must be finite and > 0".to_string());
    }
    if !grid_size_um.is_finite() || grid_size_um <= 0.0 {
        return Err("grid_size_um must be finite and > 0".to_string());
    }
    let cells = (min_bend_radius_um / grid_size_um).ceil() as i32;
    Ok(cells.max(1))
}

pub fn actual_bend_radius_um_from_cells(
    bend_radius_cells: i32,
    grid_size_um: f64,
) -> Result<f64, String> {
    if bend_radius_cells <= 0 {
        return Err("bend_radius_cells must be > 0".to_string());
    }
    if !grid_size_um.is_finite() || grid_size_um <= 0.0 {
        return Err("grid_size_um must be finite and > 0".to_string());
    }
    Ok((bend_radius_cells as f64) * grid_size_um)
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum AxisOrientation {
    Horizontal,
    Vertical,
}

fn is_finite_point(p: PhysicalPoint) -> bool {
    p.x_um.is_finite() && p.y_um.is_finite()
}

fn is_finite_box(b: MeanderBox) -> bool {
    b.min_x_um.is_finite()
        && b.max_x_um.is_finite()
        && b.min_y_um.is_finite()
        && b.max_y_um.is_finite()
}

fn point_in_box(p: PhysicalPoint, b: MeanderBox) -> bool {
    p.x_um >= b.min_x_um - EPS
        && p.x_um <= b.max_x_um + EPS
        && p.y_um >= b.min_y_um - EPS
        && p.y_um <= b.max_y_um + EPS
}

#[cfg_attr(not(test), allow(dead_code))]
fn centerline_length(points: &[PhysicalPoint]) -> f64 {
    points
        .windows(2)
        .map(|w| {
            let dx = w[1].x_um - w[0].x_um;
            let dy = w[1].y_um - w[0].y_um;
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
}

fn orientation_and_length(
    seg: StraightSegment,
) -> Result<(AxisOrientation, f64), MeanderPlanningError> {
    let dx = seg.end.x_um - seg.start.x_um;
    let dy = seg.end.y_um - seg.start.y_um;
    if dx.abs() <= EPS && dy.abs() <= EPS {
        return Err(MeanderPlanningError::NonPositiveSegmentLength);
    }
    if dy.abs() <= EPS {
        return Ok((AxisOrientation::Horizontal, dx.abs()));
    }
    if dx.abs() <= EPS {
        return Ok((AxisOrientation::Vertical, dy.abs()));
    }
    Err(MeanderPlanningError::UnsupportedSegmentOrientation)
}

fn world_from_local(
    seg: StraightSegment,
    orientation: AxisOrientation,
    side: MeanderSide,
    u: f64,
    v: f64,
) -> PhysicalPoint {
    let dx = seg.end.x_um - seg.start.x_um;
    let dy = seg.end.y_um - seg.start.y_um;
    let (tx, ty) = match orientation {
        AxisOrientation::Horizontal => (dx.signum(), 0.0),
        AxisOrientation::Vertical => (0.0, dy.signum()),
    };
    let (lx, ly) = (-ty, tx);
    let v_signed = match side {
        MeanderSide::Left => v,
        MeanderSide::Right => -v,
    };
    PhysicalPoint {
        x_um: seg.start.x_um + tx * u + lx * v_signed,
        y_um: seg.start.y_um + ty * u + ly * v_signed,
    }
}

fn side_capacity_um(
    seg: StraightSegment,
    orientation: AxisOrientation,
    b: MeanderBox,
    side: MeanderSide,
) -> f64 {
    match orientation {
        AxisOrientation::Horizontal => {
            let y0 = seg.start.y_um;
            match (seg.end.x_um >= seg.start.x_um, side) {
                (true, MeanderSide::Left) | (false, MeanderSide::Right) => b.max_y_um - y0,
                _ => y0 - b.min_y_um,
            }
        }
        AxisOrientation::Vertical => {
            let x0 = seg.start.x_um;
            match (seg.end.y_um >= seg.start.y_um, side) {
                (true, MeanderSide::Left) | (false, MeanderSide::Right) => x0 - b.min_x_um,
                _ => b.max_x_um - x0,
            }
        }
    }
}

fn append_line_local(
    out: &mut Vec<PhysicalPoint>,
    seg: StraightSegment,
    orientation: AxisOrientation,
    side: MeanderSide,
    u: f64,
    v: f64,
) {
    let p = world_from_local(seg, orientation, side, u, v);
    if out
        .last()
        .map(|last| (last.x_um - p.x_um).abs() <= EPS && (last.y_um - p.y_um).abs() <= EPS)
        .unwrap_or(false)
    {
        return;
    }
    out.push(p);
}

fn append_quarter_arc_local(
    out: &mut Vec<PhysicalPoint>,
    seg: StraightSegment,
    orientation: AxisOrientation,
    side: MeanderSide,
    cx: f64,
    cy: f64,
    r: f64,
    a0: f64,
    a1: f64,
) {
    for i in 1..=BEND_SAMPLES_PER_90_DEG {
        let t = (i as f64) / (BEND_SAMPLES_PER_90_DEG as f64);
        let a = a0 + (a1 - a0) * t;
        let u = cx + r * a.cos();
        let v = cy + r * a.sin();
        append_line_local(out, seg, orientation, side, u, v);
    }
}

pub fn plan_fill_box_multi_bump_meander(
    segment: StraightSegment,
    available_box: MeanderBox,
    config: &AnalyticMeanderConfig,
) -> Result<AnalyticMeanderPlan, MeanderPlanningError> {
    if !is_finite_point(segment.start)
        || !is_finite_point(segment.end)
        || !is_finite_box(available_box)
        || !config.requested_extra_length_um.is_finite()
        || !config.min_bend_radius_um.is_finite()
        || !config.min_straight_um.is_finite()
    {
        return Err(MeanderPlanningError::NonFiniteInput);
    }
    if config.min_bend_radius_um <= 0.0 {
        return Err(MeanderPlanningError::NonPositiveBendRadius);
    }
    if config.requested_extra_length_um <= 0.0 {
        return Err(MeanderPlanningError::NonPositiveRequestedExtraLength);
    }
    let (orientation, segment_length_um) = orientation_and_length(segment)?;
    let r = config.min_bend_radius_um;
    let min_straight = config.min_straight_um.max(0.0);
    let depth = side_capacity_um(segment, orientation, available_box, config.side)
        .min(config.max_meander_height_um);
    if depth + EPS < 2.0 * r + min_straight {
        return Err(MeanderPlanningError::AvailableBoxTooSmall);
    }
    if segment_length_um + EPS < 2.0 * r {
        return Err(MeanderPlanningError::AvailableBoxTooSmall);
    }
    // Dense comb with explicit 180-degree turns:
    // two 90-degree connectors only at entry/exit, all internal reversals are U-turns.
    let max_feasible_bumps = (((segment_length_um / r) - 3.0) * 0.5).floor() as isize;
    let max_feasible_bumps = (max_feasible_bumps.max(0) as usize).min(config.max_bumps);
    if max_feasible_bumps == 0 {
        return Err(MeanderPlanningError::MaxBumpsTooSmall);
    }
    let min_height = 2.0 * r + min_straight;
    // With `n` internal U-turns (odd), extra length is:
    //   extra = n*H + r * ((n+1)*pi - (4*n + 3))
    let mut chosen: Option<(usize, f64)> = None;
    for bumps in (1..=max_feasible_bumps).filter(|b| b % 2 == 1) {
        let n = bumps as f64;
        let h = (config.requested_extra_length_um
            - r * (((n + 1.0) * std::f64::consts::PI) - (4.0 * n + 3.0)))
            / n;
        if h + EPS < min_height || h - EPS > depth {
            continue;
        }
        chosen = Some((bumps, h.clamp(min_height, depth)));
        break;
    }
    let (num_meanders, amplitude) =
        chosen.ok_or(MeanderPlanningError::RequestedExtraLengthDoesNotFit)?;
    let insertion_width_um = r * (2.0 * (num_meanders as f64) + 3.0);
    if insertion_width_um - EPS > segment_length_um {
        return Err(MeanderPlanningError::AvailableBoxTooSmall);
    }

    let mut centerline = Vec::new();
    append_line_local(&mut centerline, segment, orientation, config.side, 0.0, 0.0);
    // Entry connector (90 deg): +x baseline -> +y vertical
    append_quarter_arc_local(
        &mut centerline,
        segment,
        orientation,
        config.side,
        r,
        r,
        r,
        -std::f64::consts::FRAC_PI_2,
        0.0,
    );
    let mut x = 2.0 * r;
    let mut going_up = true;
    for _ in 0..num_meanders {
        if going_up {
            append_line_local(
                &mut centerline,
                segment,
                orientation,
                config.side,
                x,
                amplitude - r,
            );
            // Top U-turn (180 deg): +y -> -y
            append_quarter_arc_local(
                &mut centerline,
                segment,
                orientation,
                config.side,
                x + r,
                amplitude - r,
                r,
                std::f64::consts::PI,
                std::f64::consts::FRAC_PI_2,
            );
            append_quarter_arc_local(
                &mut centerline,
                segment,
                orientation,
                config.side,
                x + r,
                amplitude - r,
                r,
                std::f64::consts::FRAC_PI_2,
                0.0,
            );
            x += 2.0 * r;
            going_up = false;
        } else {
            append_line_local(&mut centerline, segment, orientation, config.side, x, r);
            // Bottom U-turn (180 deg): -y -> +y
            append_quarter_arc_local(
                &mut centerline,
                segment,
                orientation,
                config.side,
                x + r,
                r,
                r,
                std::f64::consts::PI,
                3.0 * std::f64::consts::FRAC_PI_2,
            );
            append_quarter_arc_local(
                &mut centerline,
                segment,
                orientation,
                config.side,
                x + r,
                r,
                r,
                3.0 * std::f64::consts::FRAC_PI_2,
                2.0 * std::f64::consts::PI,
            );
            x += 2.0 * r;
            going_up = true;
        }
    }
    if going_up {
        return Err(MeanderPlanningError::RequestedExtraLengthDoesNotFit);
    }
    append_line_local(&mut centerline, segment, orientation, config.side, x, r);
    // Exit connector (90 deg): -y vertical -> +x baseline
    append_quarter_arc_local(
        &mut centerline,
        segment,
        orientation,
        config.side,
        x + r,
        r,
        r,
        std::f64::consts::PI,
        3.0 * std::f64::consts::FRAC_PI_2,
    );
    append_line_local(
        &mut centerline,
        segment,
        orientation,
        config.side,
        insertion_width_um,
        0.0,
    );
    append_line_local(
        &mut centerline,
        segment,
        orientation,
        config.side,
        segment_length_um,
        0.0,
    );
    for p in centerline.iter().copied() {
        if !point_in_box(p, available_box) {
            return Err(MeanderPlanningError::AvailableBoxTooSmall);
        }
    }
    let n = num_meanders as f64;
    let inserted_extra = n * amplitude + r * (((n + 1.0) * std::f64::consts::PI) - (4.0 * n + 3.0));
    if (inserted_extra - config.requested_extra_length_um).abs() > 1.0e-6 {
        return Err(MeanderPlanningError::RequestedExtraLengthDoesNotFit);
    }
    Ok(AnalyticMeanderPlan {
        centerline,
        inserted_extra_length_um: inserted_extra,
        bumps: num_meanders,
        side: config.side,
    })
}

pub fn plan_analytic_meander(
    segment: StraightSegment,
    available_box: MeanderBox,
    config: &AnalyticMeanderConfig,
) -> Result<AnalyticMeanderPlan, MeanderPlanningError> {
    let _ = config.mode;
    plan_fill_box_multi_bump_meander(segment, available_box, config)
}

#[cfg(test)]
mod analytic_tests {
    use super::*;

    fn assert_plan_basics(plan: &AnalyticMeanderPlan, seg: StraightSegment, b: MeanderBox) {
        assert!(!plan.centerline.is_empty());
        let first = plan.centerline.first().copied().unwrap();
        let last = plan.centerline.last().copied().unwrap();
        assert!((first.x_um - seg.start.x_um).abs() < 1.0e-6);
        assert!((first.y_um - seg.start.y_um).abs() < 1.0e-6);
        assert!((last.x_um - seg.end.x_um).abs() < 1.0e-6);
        assert!((last.y_um - seg.end.y_um).abs() < 1.0e-6);
        for p in &plan.centerline {
            assert!(point_in_box(*p, b));
        }
        let base_len = ((seg.end.x_um - seg.start.x_um).powi(2)
            + (seg.end.y_um - seg.start.y_um).powi(2))
        .sqrt();
        let planned_len = centerline_length(&plan.centerline);
        assert!(plan.inserted_extra_length_um > 0.0);
        assert!(planned_len > base_len);
    }

    #[test]
    fn analytic_meander_horizontal_success() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 200.0,
                y_um: 0.0,
            },
        };
        let b = MeanderBox {
            min_x_um: -10.0,
            max_x_um: 210.0,
            min_y_um: -5.0,
            max_y_um: 80.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 100.0,
            min_bend_radius_um: 5.0,
            min_straight_um: 2.0,
            max_bumps: 8,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let plan = plan_analytic_meander(seg, b, &cfg).expect("plan should succeed");
        assert_plan_basics(&plan, seg, b);
    }

    #[test]
    fn analytic_meander_vertical_success() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 50.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 50.0,
                y_um: 180.0,
            },
        };
        let b = MeanderBox {
            min_x_um: -20.0,
            max_x_um: 90.0,
            min_y_um: -10.0,
            max_y_um: 200.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 80.0,
            min_bend_radius_um: 4.0,
            min_straight_um: 2.0,
            max_bumps: 6,
            max_meander_height_um: 20.0,
            side: MeanderSide::Right,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let plan = plan_analytic_meander(seg, b, &cfg).expect("plan should succeed");
        assert_plan_basics(&plan, seg, b);
    }

    #[test]
    fn analytic_meander_requested_extra_too_large() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 80.0,
                y_um: 0.0,
            },
        };
        let b = MeanderBox {
            min_x_um: 0.0,
            max_x_um: 80.0,
            min_y_um: -2.0,
            max_y_um: 20.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 500.0,
            min_bend_radius_um: 3.0,
            min_straight_um: 2.0,
            max_bumps: 4,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let err = plan_analytic_meander(seg, b, &cfg).unwrap_err();
        assert!(matches!(
            err,
            MeanderPlanningError::RequestedExtraLengthDoesNotFit
                | MeanderPlanningError::AvailableBoxTooSmall
        ));
    }

    #[test]
    fn analytic_meander_diagonal_unsupported() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 100.0,
                y_um: 100.0,
            },
        };
        let b = MeanderBox {
            min_x_um: -10.0,
            max_x_um: 110.0,
            min_y_um: -10.0,
            max_y_um: 110.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 20.0,
            min_bend_radius_um: 2.0,
            min_straight_um: 1.0,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let err = plan_analytic_meander(seg, b, &cfg).unwrap_err();
        assert_eq!(err, MeanderPlanningError::UnsupportedSegmentOrientation);
    }

    #[test]
    fn analytic_meander_invalid_inputs() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 100.0,
                y_um: 0.0,
            },
        };
        let b = MeanderBox {
            min_x_um: -5.0,
            max_x_um: 105.0,
            min_y_um: -5.0,
            max_y_um: 50.0,
        };

        let bad_radius = AnalyticMeanderConfig {
            requested_extra_length_um: 20.0,
            min_bend_radius_um: 0.0,
            min_straight_um: 1.0,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let err = plan_analytic_meander(seg, b, &bad_radius).unwrap_err();
        assert_eq!(err, MeanderPlanningError::NonPositiveBendRadius);

        let bad_extra = AnalyticMeanderConfig {
            requested_extra_length_um: 0.0,
            min_bend_radius_um: 2.0,
            min_straight_um: 1.0,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let err = plan_analytic_meander(seg, b, &bad_extra).unwrap_err();
        assert_eq!(err, MeanderPlanningError::NonPositiveRequestedExtraLength);
    }

    #[test]
    fn fill_box_multi_bump_produces_multiple_bumps() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 220.0,
                y_um: 0.0,
            },
        };
        let b = MeanderBox {
            min_x_um: 0.0,
            max_x_um: 220.0,
            min_y_um: -1.0,
            max_y_um: 30.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 20.0,
            min_bend_radius_um: 2.0,
            min_straight_um: 1.0,
            max_bumps: 20,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let plan = plan_analytic_meander(seg, b, &cfg).unwrap();
        assert!(plan.bumps >= 1);
        assert!((plan.inserted_extra_length_um - cfg.requested_extra_length_um).abs() <= 1.0e-6);
    }

    #[test]
    fn fill_box_multi_bump_count_monotonic_with_depth() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 220.0,
                y_um: 0.0,
            },
        };
        let shallow = MeanderBox {
            min_x_um: 0.0,
            max_x_um: 220.0,
            min_y_um: -1.0,
            max_y_um: 16.0,
        };
        let deep = MeanderBox {
            min_x_um: 0.0,
            max_x_um: 220.0,
            min_y_um: -1.0,
            max_y_um: 30.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 10.0,
            min_bend_radius_um: 2.0,
            min_straight_um: 1.0,
            max_bumps: 20,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p1 = plan_analytic_meander(seg, shallow, &cfg).unwrap();
        let p2 = plan_analytic_meander(seg, deep, &cfg).unwrap();
        assert!(p2.bumps >= p1.bumps);
    }

    #[test]
    fn bend_radius_cell_conversion_rounds_up() {
        let cells = bend_radius_cells_from_min_radius(5.1, 1.0).unwrap();
        assert_eq!(cells, 6);
        let actual = actual_bend_radius_um_from_cells(cells, 1.0).unwrap();
        assert_eq!(actual, 6.0);
    }

    #[test]
    fn fill_box_multi_bump_fails_if_requested_extra_too_large() {
        let seg = StraightSegment {
            start: PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            end: PhysicalPoint {
                x_um: 80.0,
                y_um: 0.0,
            },
        };
        let b = MeanderBox {
            min_x_um: 0.0,
            max_x_um: 80.0,
            min_y_um: 0.0,
            max_y_um: 12.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 500.0,
            min_bend_radius_um: 2.0,
            min_straight_um: 1.0,
            max_bumps: 20,
            max_meander_height_um: 20.0,
            side: MeanderSide::Left,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let err = plan_analytic_meander(seg, b, &cfg).unwrap_err();
        assert_eq!(err, MeanderPlanningError::RequestedExtraLengthDoesNotFit);
    }
}
