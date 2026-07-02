//! Synthetic world / scan / trajectory fixtures for SLAM tests (plan T6).
//!
//! Compiled only for tests. Generates `LidarPoint` sweeps by ray-casting a
//! segment-based world, with the failure modes of the real sensor dialed in
//! as parameters: angular gap sectors (issues.md #7), range noise, and
//! corrupted odometry (issues.md #6). All randomness is a seeded LCG so
//! every test run is reproducible.

use crate::homerobot::LidarPoint;
use crate::odometry::Pose;

/// Deterministic LCG; same constants as the server-side fault injection.
pub struct Lcg(pub u32);

impl Lcg {
    pub fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        self.0
    }

    /// Uniform in [-1, 1).
    pub fn next_signed(&mut self) -> f32 {
        ((self.next_u32() >> 8) as f32 / (1u32 << 24) as f32) * 2.0 - 1.0
    }
}

/// Wall-segment world; the unit for everything is meters.
pub struct SyntheticWorld {
    pub segments: Vec<((f32, f32), (f32, f32))>,
}

impl SyntheticWorld {
    /// Empty square arena with walls at ±`half` (compare simulation/sim.world).
    pub fn arena(half: f32) -> Self {
        let h = half;
        Self {
            segments: vec![
                ((-h, -h), (h, -h)),
                ((h, -h), (h, h)),
                ((h, h), (-h, h)),
                ((-h, h), (-h, -h)),
            ],
        }
    }

    /// Like [`cluttered_arena`] but with the octagon moved off the origin
    /// (to (1.5, -2.0)) so the world origin is clear floor. Engine-level
    /// tests MUST use this: a `GridSlam`'s frame is rooted wherever its
    /// first sweep happens, and starting tests at the world origin is the
    /// only way to keep SLAM frame == world frame without transform
    /// bookkeeping. (`cluttered_arena`'s cylinder sits ON the origin — a
    /// robot there sees only a 0.5m ring and rotation is unobservable.)
    pub fn test_room(half: f32) -> Self {
        let mut world = Self::cluttered_arena(half);
        // Drop the 8 octagon segments (added last) and re-add displaced.
        let n = world.segments.len();
        world.segments.truncate(n - 8);
        let r = 0.5;
        let (cx, cy) = (1.5, -2.0);
        for i in 0..8 {
            let a0 = std::f32::consts::TAU * i as f32 / 8.0;
            let a1 = std::f32::consts::TAU * (i + 1) as f32 / 8.0;
            world.segments.push((
                (cx + r * a0.cos(), cy + r * a0.sin()),
                (cx + r * a1.cos(), cy + r * a1.sin()),
            ));
        }
        world
    }

    /// Arena plus interior structure similar to the sim world: a diagonal
    /// wall, a box, and an octagon standing in for the cylinder.
    pub fn cluttered_arena(half: f32) -> Self {
        let mut world = Self::arena(half);
        // 2m wall centered (2,2) at 45°.
        let c = std::f32::consts::FRAC_1_SQRT_2;
        world.segments.push(((2.0 - c, 2.0 - c), (2.0 + c, 2.0 + c)));
        // 0.5 x 3.0 box centered (-2,-2).
        let (bx0, bx1, by0, by1) = (-2.25, -1.75, -3.5, -0.5);
        world.segments.push(((bx0, by0), (bx1, by0)));
        world.segments.push(((bx1, by0), (bx1, by1)));
        world.segments.push(((bx1, by1), (bx0, by1)));
        world.segments.push(((bx0, by1), (bx0, by0)));
        // r=0.5 cylinder at the origin, as an octagon.
        let r = 0.5;
        for i in 0..8 {
            let a0 = std::f32::consts::TAU * i as f32 / 8.0;
            let a1 = std::f32::consts::TAU * (i + 1) as f32 / 8.0;
            world
                .segments
                .push(((r * a0.cos(), r * a0.sin()), (r * a1.cos(), r * a1.sin())));
        }
        world
    }

    /// Distance from (x, y) along world angle `angle` to the nearest wall.
    pub fn raycast(&self, x: f32, y: f32, angle: f32, max_range: f32) -> Option<f32> {
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best: Option<f32> = None;
        for &((ax, ay), (bx, by)) in &self.segments {
            let (ex, ey) = (bx - ax, by - ay);
            let denom = dx * ey - dy * ex;
            if denom.abs() < 1e-9 {
                continue; // parallel
            }
            // Solve origin + t*d == a + u*e.
            let t = ((ax - x) * ey - (ay - y) * ex) / denom;
            let u = ((ax - x) * dy - (ay - y) * dx) / denom;
            if t > 1e-4 && (0.0..=1.0).contains(&u) && t <= max_range {
                best = Some(best.map_or(t, |b: f32| b.min(t)));
            }
        }
        best
    }

    /// Distance from a point to the nearest wall segment (for assertions).
    pub fn dist_to_walls(&self, x: f32, y: f32) -> f32 {
        let mut best = f32::MAX;
        for &((ax, ay), (bx, by)) in &self.segments {
            let (ex, ey) = (bx - ax, by - ay);
            let len2 = ex * ex + ey * ey;
            let t = (((x - ax) * ex + (y - ay) * ey) / len2).clamp(0.0, 1.0);
            let (px, py) = (ax + t * ex, ay + t * ey);
            best = best.min((x - px).hypot(y - py));
        }
        best
    }
}

/// Sensor model knobs; defaults mirror the real RPLidar path in `slam.rs`
/// (0.2–8m band) and a 360-points-per-revolution sweep.
pub struct ScanConfig {
    pub points_per_rev: usize,
    /// Missing angular sectors in lidar-frame degrees `[start, end)`.
    pub gap_sectors: Vec<(f32, f32)>,
    /// Uniform range noise, ± meters.
    pub range_noise_m: f32,
    pub min_range_m: f32,
    pub max_range_m: f32,
}

impl Default for ScanConfig {
    fn default() -> Self {
        Self {
            points_per_rev: 360,
            gap_sectors: Vec::new(),
            range_noise_m: 0.0,
            min_range_m: 0.2,
            max_range_m: 8.0,
        }
    }
}

/// One full revolution captured at a fixed pose. The angle convention matches
/// `BasicSlam::get_cartesian_points`: a point at lidar angle `a` lies along
/// world angle `pose.theta - a`.
pub fn sweep_at(world: &SyntheticWorld, pose: &Pose, cfg: &ScanConfig, rng: &mut Lcg) -> Vec<LidarPoint> {
    let mut points = Vec::with_capacity(cfg.points_per_rev);
    for i in 0..cfg.points_per_rev {
        let angle_deg = 360.0 * i as f32 / cfg.points_per_rev as f32;
        if cfg
            .gap_sectors
            .iter()
            .any(|&(s, e)| angle_deg >= s && angle_deg < e)
        {
            continue;
        }
        let world_angle = pose.theta - angle_deg.to_radians();
        let Some(mut dist) = world.raycast(pose.x, pose.y, world_angle, cfg.max_range_m) else {
            continue;
        };
        dist += cfg.range_noise_m * rng.next_signed();
        if dist < cfg.min_range_m || dist > cfg.max_range_m {
            continue;
        }
        points.push(LidarPoint {
            distance_mm: dist * 1000.0,
            angle_deg,
            quality: 15,
            scan_completed: false,
        });
    }
    points
}

/// Ground-truth robot trajectory sampled at a fixed rate.
pub struct Trajectory {
    pub poses: Vec<Pose>,
}

impl Trajectory {
    /// Waypoint patrol: turn in place toward each waypoint, then drive
    /// straight to it. `speed` m/s, `turn_rate` rad/s, sampled at `hz`.
    pub fn patrol(start: Pose, waypoints: &[(f32, f32)], speed: f32, turn_rate: f32, hz: f32) -> Self {
        let mut poses = vec![start];
        let dt = 1.0 / hz;
        for &(wx, wy) in waypoints {
            // Rotate toward the waypoint.
            loop {
                let cur = *poses.last().unwrap();
                let target = (wy - cur.y).atan2(wx - cur.x);
                let mut err = target - cur.theta;
                while err > std::f32::consts::PI {
                    err -= std::f32::consts::TAU;
                }
                while err < -std::f32::consts::PI {
                    err += std::f32::consts::TAU;
                }
                if err.abs() < turn_rate * dt {
                    poses.push(Pose { theta: target, ..cur });
                    break;
                }
                poses.push(Pose {
                    theta: cur.theta + err.signum() * turn_rate * dt,
                    ..cur
                });
            }
            // Drive to the waypoint.
            loop {
                let cur = *poses.last().unwrap();
                let dist = (wx - cur.x).hypot(wy - cur.y);
                if dist < speed * dt {
                    poses.push(Pose { x: wx, y: wy, ..cur });
                    break;
                }
                poses.push(Pose {
                    x: cur.x + speed * dt * cur.theta.cos(),
                    y: cur.y + speed * dt * cur.theta.sin(),
                    ..cur
                });
            }
        }
        Self { poses }
    }

    /// P2-style rectangular loop from the origin, returning to it. Meant
    /// for [`SyntheticWorld::test_room`], whose origin is clear floor.
    pub fn p2_loop(hz: f32) -> Self {
        Self::patrol(
            Pose { x: 0.0, y: 0.0, theta: 0.0 },
            &[(3.0, 3.0), (-3.0, 3.0), (-3.0, -3.0), (3.0, -3.0), (0.0, 0.0)],
            0.25,
            0.8,
            hz,
        )
    }
}

/// Systematic odometry corruption applied to a ground-truth trajectory,
/// mimicking encoder faults at the pose level.
pub struct OdomCorruption {
    /// Heading drift per meter traveled (rad/m).
    pub heading_drift_rad_per_m: f32,
    /// Translation scale error (1.0 = perfect).
    pub scale: f32,
    /// Phantom forward creep per sample while stationary (m), like the
    /// issues.md #6 idle ticks.
    pub idle_creep_m: f32,
}

/// Integrates the ground-truth deltas through the corruption model, returning
/// what the wheel odometry would have reported.
pub fn corrupt_odometry(truth: &[Pose], cfg: &OdomCorruption) -> Vec<Pose> {
    let mut out = Vec::with_capacity(truth.len());
    let mut cur = truth[0];
    out.push(cur);
    for pair in truth.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let step = (b.x - a.x).hypot(b.y - a.y);
        let mut dtheta = b.theta - a.theta;
        while dtheta > std::f32::consts::PI {
            dtheta -= std::f32::consts::TAU;
        }
        while dtheta < -std::f32::consts::PI {
            dtheta += std::f32::consts::TAU;
        }
        let dist = step * cfg.scale + if step == 0.0 { cfg.idle_creep_m } else { 0.0 };
        cur.theta += dtheta + cfg.heading_drift_rad_per_m * step;
        cur.x += dist * cur.theta.cos();
        cur.y += dist * cur.theta.sin();
        out.push(cur);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// T6 acceptance: points projected at ground-truth poses must land on
    /// the walls — >=95% within one grid cell (5cm), for every pose of a
    /// P2-like loop, including gap sectors and noise.
    #[test]
    fn sweeps_projected_at_truth_land_on_walls() {
        let world = SyntheticWorld::test_room(5.0);
        let traj = Trajectory::p2_loop(5.0);
        let cfg = ScanConfig {
            gap_sectors: vec![(100.0, 130.0)], // issues.md #7-style missing fan
            range_noise_m: 0.01,
            ..ScanConfig::default()
        };
        let mut rng = Lcg(42);

        assert!(traj.poses.len() > 100, "loop must be long enough to matter");
        let mut total = 0usize;
        let mut on_wall = 0usize;
        for pose in traj.poses.iter().step_by(7) {
            let sweep = sweep_at(&world, pose, &cfg, &mut rng);
            assert!(sweep.len() > 100, "sweep too sparse at ({}, {})", pose.x, pose.y);
            for p in &sweep {
                let world_angle = pose.theta - p.angle_deg.to_radians();
                let d = p.distance_mm / 1000.0;
                let (x, y) = (pose.x + d * world_angle.cos(), pose.y + d * world_angle.sin());
                total += 1;
                if world.dist_to_walls(x, y) <= 0.05 {
                    on_wall += 1;
                }
            }
        }
        let pct = 100.0 * on_wall as f32 / total as f32;
        assert!(pct >= 95.0, "only {:.1}% of {} points landed on walls", pct, total);
    }

    #[test]
    fn gap_sectors_remove_exactly_that_fan() {
        let world = SyntheticWorld::arena(5.0);
        let pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let mut rng = Lcg(1);
        let clean = sweep_at(&world, &pose, &ScanConfig::default(), &mut rng);
        let gapped = sweep_at(
            &world,
            &pose,
            &ScanConfig { gap_sectors: vec![(90.0, 120.0)], ..ScanConfig::default() },
            &mut rng,
        );
        assert_eq!(clean.len() - gapped.len(), 30, "30 of 360 one-degree steps fall in the fan");
        assert!(gapped.iter().all(|p| !(90.0..120.0).contains(&p.angle_deg)));
    }

    #[test]
    fn patrol_reaches_waypoints_and_returns() {
        let traj = Trajectory::p2_loop(5.0);
        let first = traj.poses[0];
        let last = traj.poses.last().unwrap();
        assert!(
            (last.x - first.x).abs() < 0.06 && (last.y - first.y).abs() < 0.06,
            "loop must close on its start"
        );
        // Steps are bounded by the commanded speed.
        for pair in traj.poses.windows(2) {
            let step = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
            assert!(step <= 0.25 / 5.0 + 1e-4);
        }
    }

    #[test]
    fn corrupted_odometry_diverges_from_truth() {
        let traj = Trajectory::p2_loop(5.0);
        let odom = corrupt_odometry(
            &traj.poses,
            &OdomCorruption { heading_drift_rad_per_m: 0.02, scale: 1.05, idle_creep_m: 0.0 },
        );
        assert_eq!(odom.len(), traj.poses.len());
        // A loop that closes on its own start hides scale error at the
        // endpoint, so judge divergence at its worst point along the path.
        let max_err = traj
            .poses
            .iter()
            .zip(&odom)
            .map(|(t, o)| (t.x - o.x).hypot(t.y - o.y))
            .fold(0.0f32, f32::max);
        assert!(max_err > 0.3, "corruption must visibly diverge, got {:.3}m", max_err);
        // Zero corruption must reproduce the truth.
        let clean = corrupt_odometry(
            &traj.poses,
            &OdomCorruption { heading_drift_rad_per_m: 0.0, scale: 1.0, idle_creep_m: 0.0 },
        );
        let (t, c) = (traj.poses.last().unwrap(), clean.last().unwrap());
        assert!((t.x - c.x).hypot(t.y - c.y) < 1e-3);
    }
}
