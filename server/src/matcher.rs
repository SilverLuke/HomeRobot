//! Multi-resolution correlative scan matcher (plan T8/T9, design doc §4.2).
//!
//! Exhaustively scores candidate poses against the likelihood field: for
//! each candidate heading the scan is rotated ONCE into grid offsets, so
//! every candidate translation is pure array lookups. Pass A absorbs a
//! garbage motion prior (the window always covers worst-case inter-sweep
//! motion) with rotation searched jointly at ~1° against 20cm-strided
//! translations on the fine raster; pass B refines at 5cm / 0.5° with
//! bilinear scoring and returns the soft-argmax (sub-cell) pose plus a
//! 3x3 covariance of the local score surface.
//!
//! Scan points arrive in the ROBOT frame (x forward at theta=0); a
//! candidate pose (theta, tx, ty) places them at `R(theta)·p + t`.

#![allow(dead_code)] // wired into GridSlam by plan T10

use crate::likelihood::{LikelihoodField, COARSE_FACTOR};
use crate::odometry::Pose;

/// Rotation step of pass A (radians): 1° — near-fine, because rotation is
/// searched jointly with the strided translation (see correlative_match).
const COARSE_PASS_THETA_STEP: f32 = 0.01745;
/// Fine angular step (radians): 0.5°.
const FINE_THETA_STEP: f32 = 0.008727;

pub struct SearchWindow {
    /// Half-width of the translation search, meters (each axis).
    pub trans_m: f32,
    /// Half-width of the rotation search, radians.
    pub rot_rad: f32,
}

/// Nominal odometry: covers worst-case inter-sweep motion at 5Hz twice over.
pub const WINDOW_NOMINAL: SearchWindow = SearchWindow { trans_m: 0.30, rot_rad: 0.14 };
/// Degraded odometry (encoder fault): zero-translation prior, wider net.
pub const WINDOW_DEGRADED: SearchWindow = SearchWindow { trans_m: 0.45, rot_rad: 0.21 };

#[derive(Clone, Copy, Debug)]
pub struct MatchResult {
    pub pose: Pose,
    /// Mean per-point field value in [0, 1]; 1.0 = every point dead-center
    /// on confident structure.
    pub score: f32,
    /// 3x3 covariance over (x, y, theta) from the fine score surface.
    /// Row-major; large diagonal terms mean that axis is unconstrained
    /// (e.g. along a featureless corridor).
    pub covariance: [[f32; 3]; 3],
    pub points: usize,
}

/// Full coarse-to-fine match. `points_robot` are robot-frame Cartesian
/// points (deskewed against the prediction upstream); `center` is the
/// motion-prior pose the window is laid around.
pub fn correlative_match(
    field: &LikelihoodField,
    points_robot: &[(f32, f32)],
    center: &Pose,
    window: &SearchWindow,
) -> Option<MatchResult> {
    if points_robot.is_empty() {
        return None;
    }
    // Pass A: joint search over near-fine rotation x strided translation,
    // scored on the FINE raster. Two hard-won constraints meet here:
    // rotation must be searched jointly with translation (a translation
    // chosen at the wrong heading absorbs rotation into a centroid shift of
    // ~range * dtheta, pushing the true optimum out of any local follow-up
    // window), and rotation cannot be discriminated on the max-decimated
    // coarse raster (its fat walls make the score flat in theta). The
    // Gaussian kernel's ~25cm basin covers the 20cm stride, so no basin is
    // skipped; the coarse raster is kept for future global relocalization.
    let pass_a = search_level(
        field,
        points_robot,
        center,
        window,
        &Stage { level: Level::FineNearest, cell_m: field.coarse_cell_m(), theta_step: COARSE_PASS_THETA_STEP },
        None,
    );
    // Pass B: fine translation around the pass-A winner, with the residual
    // rotation range. Pass A's heading error is at most its own step, so the
    // leftover rotation-translation coupling fits inside one coarse cell.
    let fine_window = SearchWindow {
        trans_m: field.coarse_cell_m(),
        rot_rad: 2.0 * COARSE_PASS_THETA_STEP,
    };
    let mut samples = Vec::new();
    let fine = search_level(
        field,
        points_robot,
        &pass_a.pose,
        &fine_window,
        &Stage { level: Level::FineBilinear, cell_m: field.fine_cell_m(), theta_step: FINE_THETA_STEP },
        Some(&mut samples),
    );
    let (soft_pose, covariance) = covariance_from_samples(&samples);
    // Soft-argmax: the score-weighted mean of the fine surface gives a
    // SUB-CELL pose estimate. Raw lattice argmax jitters by up to half a
    // cell per sweep; painting that jitter into the map smears the walls
    // and the matcher then chases its own smear (accumulating random walk).
    // Guard against multimodal pull: if the mean strays more than a cell
    // from the argmax, trust the argmax.
    let pose = match soft_pose {
        Some(p)
            if (p.x - fine.pose.x).hypot(p.y - fine.pose.y) <= 1.5 * field.fine_cell_m()
                && (p.theta - fine.pose.theta).abs() <= 2.0 * FINE_THETA_STEP =>
        {
            p
        }
        _ => fine.pose,
    };
    Some(MatchResult {
        pose,
        score: fine.score,
        covariance,
        points: points_robot.len(),
    })
}

#[derive(Clone, Copy)]
enum Level {
    /// Nearest-cell lookups on the max-decimated coarse raster. Unused by
    /// the tracking path (theta is indiscriminable on fat coarse walls);
    /// reserved for a future whole-map relocalization sweep.
    #[allow(dead_code)]
    CoarseNearest,
    /// Nearest-cell lookups on the fine raster: cheap, used for pass A.
    FineNearest,
    /// Bilinear interpolation on the fine raster: sub-cell score accuracy
    /// for the refinement pass.
    FineBilinear,
}

struct LevelResult {
    pose: Pose,
    score: f32,
}

/// (pose, normalized score) samples of one level's search, for covariance.
type Sample = (f32, f32, f32, f32); // x, y, theta, score

/// One search stage: which raster/lookup to score with, the translation
/// stride, and the rotation step.
struct Stage {
    level: Level,
    cell_m: f32,
    theta_step: f32,
}

fn search_level(
    field: &LikelihoodField,
    points: &[(f32, f32)],
    center: &Pose,
    window: &SearchWindow,
    stage: &Stage,
    mut samples: Option<&mut Vec<Sample>>,
) -> LevelResult {
    let Stage { level, cell_m, theta_step } = *stage;
    let n_trans = (window.trans_m / cell_m).ceil() as i32;
    let n_theta = (window.rot_rad / theta_step).ceil() as i32;
    let max_sum = (255usize * points.len()) as f32;

    // Zero-motion prior: on a flat score plateau (sparse map, aliased
    // structure) the argmax must prefer candidates near the prediction, not
    // whichever window corner raster order visits first. The penalty is a
    // few percent at the window edge — real score gradients dominate it.
    const CENTER_BIAS: f32 = 0.04;
    let trans_span = (n_trans as f32 * cell_m).max(1e-6);
    let rot_span = (n_theta as f32 * theta_step).max(1e-6);

    let mut best = LevelResult { pose: *center, score: -1.0 };
    let mut best_penalized = f32::MIN;
    for ti in -n_theta..=n_theta {
        let theta = center.theta + ti as f32 * theta_step;
        let (sin_t, cos_t) = theta.sin_cos();
        // Rotate + translate to the window center once; candidate
        // translations then shift by whole cells in grid space.
        let base: Vec<(f32, f32)> = points
            .iter()
            .map(|&(px, py)| {
                let wx = center.x + px * cos_t - py * sin_t;
                let wy = center.y + px * sin_t + py * cos_t;
                field.world_to_fine_f(wx, wy)
            })
            .collect();
        for dy in -n_trans..=n_trans {
            for dx in -n_trans..=n_trans {
                // Cell shift in fine-grid units.
                let shift = cell_m / field.fine_cell_m();
                let (fx_off, fy_off) = (dx as f32 * shift, dy as f32 * shift);
                let mut sum = 0.0f32;
                match level {
                    Level::CoarseNearest => {
                        for &(fx, fy) in &base {
                            // floor, not truncate: -0.5 must not alias cell 0
                            let gx = (fx + fx_off).floor() as i32;
                            let gy = (fy + fy_off).floor() as i32;
                            if gx >= 0 && gy >= 0 {
                                sum += field
                                    .coarse_at(gx as usize / COARSE_FACTOR, gy as usize / COARSE_FACTOR)
                                    as f32;
                            }
                        }
                    }
                    Level::FineNearest => {
                        for &(fx, fy) in &base {
                            let gx = (fx + fx_off).floor() as i32;
                            let gy = (fy + fy_off).floor() as i32;
                            if gx >= 0 && gy >= 0 {
                                sum += field.fine_at(gx as usize, gy as usize) as f32;
                            }
                        }
                    }
                    Level::FineBilinear => {
                        for &(fx, fy) in &base {
                            sum += field.fine_bilinear(fx + fx_off, fy + fy_off);
                        }
                    }
                }
                let score = sum / max_sum;
                let (cx, cy) = (center.x + dx as f32 * cell_m, center.y + dy as f32 * cell_m);
                if let Some(s) = samples.as_deref_mut() {
                    s.push((cx, cy, theta, score));
                }
                let d_trans = ((dx as f32 * cell_m).powi(2) + (dy as f32 * cell_m).powi(2))
                    / trans_span.powi(2);
                let d_rot = (ti as f32 * theta_step / rot_span).powi(2);
                let penalized = score - CENTER_BIAS * (d_trans + d_rot);
                if penalized > best_penalized {
                    best_penalized = penalized;
                    best = LevelResult {
                        pose: Pose { x: cx, y: cy, theta },
                        score,
                    };
                }
            }
        }
    }
    best
}

/// Weighted mean (soft-argmax, sub-cell pose) and covariance of the fine
/// score surface. Weights follow Olson: exp(beta * (score - best)) —
/// candidates nearly as good as the winner spread the distribution along
/// the unconstrained axis.
fn covariance_from_samples(samples: &[Sample]) -> (Option<Pose>, [[f32; 3]; 3]) {
    let best = match samples.iter().map(|s| s.3).fold(None, |acc: Option<f32>, s| {
        Some(acc.map_or(s, |a| a.max(s)))
    }) {
        Some(b) => b,
        None => return (None, [[0.0; 3]; 3]),
    };
    const BETA: f32 = 60.0;
    let mut w_sum = 0.0;
    let mut mean = [0.0f32; 3];
    for &(x, y, t, s) in samples {
        let w = (BETA * (s - best)).exp();
        w_sum += w;
        mean[0] += w * x;
        mean[1] += w * y;
        mean[2] += w * t;
    }
    if w_sum <= 0.0 {
        return (None, [[0.0; 3]; 3]);
    }
    for m in &mut mean {
        *m /= w_sum;
    }
    let mut cov = [[0.0f32; 3]; 3];
    for &(x, y, t, s) in samples {
        let w = (BETA * (s - best)).exp();
        let d = [x - mean[0], y - mean[1], t - mean[2]];
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] += w * d[i] * d[j];
            }
        }
    }
    for row in &mut cov {
        for v in row.iter_mut() {
            *v /= w_sum;
        }
    }
    (Some(Pose { x: mean[0], y: mean[1], theta: mean[2] }), cov)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::likelihood::LikelihoodField;
    use crate::mapping::OccupancyGrid;
    use crate::test_fixtures::{sweep_at, Lcg, ScanConfig, SyntheticWorld};
    use std::time::Instant;

    /// Paints a grid + field from sweeps taken around `pose` in `world`.
    /// Several vantage points, like a real map accumulated at 5Hz — a
    /// single-sweep map is dotted walls, unrealistically weak structure.
    fn field_from_pose(world: &SyntheticWorld, pose: &Pose) -> LikelihoodField {
        let mut grid = OccupancyGrid::new(600, 600, 0.05);
        let mut rng = Lcg(7);
        for (jx, jy, jt) in [(0.0, 0.0, 0.0), (0.06, -0.04, 0.05), (-0.05, 0.07, -0.04)] {
            let vantage = Pose { x: pose.x + jx, y: pose.y + jy, theta: pose.theta + jt };
            let sweep = sweep_at(world, &vantage, &ScanConfig::default(), &mut rng);
            for p in &sweep {
                let a = vantage.theta - p.angle_deg.to_radians();
                let d = p.distance_mm / 1000.0;
                grid.update_cell(vantage.x + d * a.cos(), vantage.y + d * a.sin(), 100);
            }
        }
        let mut field = LikelihoodField::new(600, 600);
        field.rebuild_full(&grid);
        field
    }

    /// Robot-frame Cartesian points of a sweep (matches slam.rs convention:
    /// global angle = theta - lidar angle => robot-frame angle = -lidar angle).
    fn robot_frame(sweep: &[crate::homerobot::LidarPoint]) -> Vec<(f32, f32)> {
        sweep
            .iter()
            .map(|p| {
                let a = -p.angle_deg.to_radians();
                let d = p.distance_mm / 1000.0;
                (d * a.cos(), d * a.sin())
            })
            .collect()
    }

    fn wrap(a: f32) -> f32 {
        let mut a = a;
        while a > std::f32::consts::PI {
            a -= std::f32::consts::TAU;
        }
        while a < -std::f32::consts::PI {
            a += std::f32::consts::TAU;
        }
        a
    }

    #[test]
    fn recovers_injected_offsets_within_fine_resolution() {
        let world = SyntheticWorld::cluttered_arena(5.0);
        let map_pose = Pose { x: -2.8, y: 2.6, theta: 0.2 };
        let field = field_from_pose(&world, &map_pose);
        let mut rng = Lcg(99);

        // The robot actually stands displaced from where odometry thinks.
        // Heading offsets stay <= 0.10 rad: the heading prior is gyro-backed
        // (98% complementary filter) even under the encoder fault, so larger
        // heading errors are outside the design envelope — and the matcher's
        // center bias deliberately resists them.
        for &(ex, ey, et) in &[
            (0.25, -0.30, 0.10),
            (-0.40, 0.15, -0.10),
            (0.10, 0.42, 0.05),
        ] {
            let true_pose = Pose { x: map_pose.x + ex, y: map_pose.y + ey, theta: map_pose.theta + et };
            let sweep = sweep_at(&world, &true_pose, &ScanConfig::default(), &mut rng);
            let points = robot_frame(&sweep);
            // Prior = the MAP pose (i.e. odometry error == the injected offset).
            let m = correlative_match(&field, &points, &map_pose, &WINDOW_DEGRADED)
                .expect("match must produce a result");
            let err = (m.pose.x - true_pose.x).hypot(m.pose.y - true_pose.y);
            let err_t = wrap(m.pose.theta - true_pose.theta).abs();
            let diag = format!(
                "offset ({}, {}, {}): matched ({:.3}, {:.3}, {:.4}) score {:.3}, true ({:.3}, {:.3}, {:.4})",
                ex, ey, et, m.pose.x, m.pose.y, m.pose.theta, m.score,
                true_pose.x, true_pose.y, true_pose.theta
            );
            assert!(err <= 0.05 + 1e-3, "translation error {:.3}m — {}", err, diag);
            assert!(err_t <= 0.0088 + 1e-4, "rotation error {:.4}rad — {}", err_t, diag);
            assert!(m.score > 0.5, "score too low — {}", diag);
        }
    }

    #[test]
    fn survives_gap_sector_and_noise() {
        let world = SyntheticWorld::cluttered_arena(5.0);
        let map_pose = Pose { x: -3.0, y: 1.0, theta: -0.4 };
        let field = field_from_pose(&world, &map_pose);
        let mut rng = Lcg(5);

        let true_pose = Pose { x: -2.8, y: 0.8, theta: -0.3 };
        let cfg = ScanConfig {
            gap_sectors: vec![(90.0, 120.0)], // 30° missing fan (issues.md #7)
            range_noise_m: 0.02,
            ..ScanConfig::default()
        };
        let sweep = sweep_at(&world, &true_pose, &cfg, &mut rng);
        let m = correlative_match(&field, &robot_frame(&sweep), &map_pose, &WINDOW_DEGRADED).unwrap();
        let err = (m.pose.x - true_pose.x).hypot(m.pose.y - true_pose.y);
        assert!(err <= 0.07, "translation error {:.3}m with gaps+noise", err);
    }

    #[test]
    fn corridor_covariance_is_elongated_along_the_corridor() {
        // Infinite corridor along X: two horizontal walls, nothing else.
        let world = SyntheticWorld {
            segments: vec![((-12.0, -1.0), (12.0, -1.0)), ((-12.0, 1.0), (12.0, 1.0))],
        };
        let pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let field = field_from_pose(&world, &pose);
        let mut rng = Lcg(3);
        let sweep = sweep_at(&world, &pose, &ScanConfig::default(), &mut rng);
        let m = correlative_match(&field, &robot_frame(&sweep), &pose, &WINDOW_NOMINAL).unwrap();

        // 2x2 position block: variance along X must dominate variance along Y.
        let (sxx, syy) = (m.covariance[0][0], m.covariance[1][1]);
        assert!(
            sxx > 5.0 * syy,
            "corridor must be unconstrained along X: sxx={:.6}, syy={:.6}",
            sxx,
            syy
        );
    }

    #[test]
    fn full_match_stays_fast_in_debug() {
        let world = SyntheticWorld::cluttered_arena(5.0);
        let pose = Pose { x: -2.5, y: 2.5, theta: 0.0 };
        let field = field_from_pose(&world, &pose);
        let mut rng = Lcg(11);
        let sweep = sweep_at(&world, &pose, &ScanConfig::default(), &mut rng);
        let points = robot_frame(&sweep);

        let start = Instant::now();
        let m = correlative_match(&field, &points, &pose, &WINDOW_DEGRADED).unwrap();
        let elapsed = start.elapsed();
        assert!(m.score > 0.5);
        // Budget: well under the 200ms sweep period even in debug.
        assert!(elapsed.as_millis() < 50, "match took {:?} (budget 50ms debug)", elapsed);
    }
}
