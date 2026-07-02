//! Map-anchored SLAM engine (plan T10, design doc §4.2).
//!
//! Per sweep: predict from the odometry delta → deskew → correlative match
//! against the likelihood field of the occupancy grid → adopt the matched
//! pose when the score clears the gate → update the map (gated on score, so
//! a lost robot cannot scribble over a good map).
//!
//! Unlike `BasicSlam`'s rolling reference cloud, the reference here IS the
//! persisted grid: revisit error is bounded by map consistency and a
//! restored map is immediately matchable after restart.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crate::homerobot::LidarPoint;
use crate::likelihood::LikelihoodField;
use crate::mapping::{cell_is_occupied, OccupancyGrid};
use crate::matcher::{correlative_match, SearchWindow, WINDOW_DEGRADED, WINDOW_NOMINAL};
use crate::odometry::Pose;
use crate::slam::{cartesian_points, deskewed_cartesian_points, Slam, SlamHealth, SlamMode};

/// Sweeps whose match score falls below this leave the grid untouched.
const MAP_UPDATE_MIN_SCORE: f32 = 0.30;
/// Below this the matched pose is not adopted either (dead-reckon instead).
const MATCH_ACCEPT_SCORE: f32 = 0.20;
/// Occupied-cell count below which the map is considered structure-less
/// (bootstrap: nothing to match against yet).
const BOOTSTRAP_MIN_OCCUPIED: usize = 40;
/// Tracking degrades to Diverged after this many consecutive sweeps below
/// [`DIVERGED_SCORE`]; Diverged/Relocalizing recover at [`RECOVER_SCORE`].
/// Calibrated against measured fixture scores: correct matches on a warm
/// map score 0.7–0.95; aliased garbage (kidnapped/wrong-map sweeps) tops
/// out around 0.17.
const DIVERGED_SCORE: f32 = 0.25;
const DIVERGED_AFTER: u32 = 3;
const RECOVER_SCORE: f32 = 0.50;
/// Consecutive good sweeps required to confirm a pose after restore.
const RELOC_CONFIRMS: u32 = 2;

/// World- or robot-frame Cartesian points of one sweep.
type PointSet = Vec<(f32, f32)>;

pub struct GridSlam {
    current_pose: Pose,
    last_odom_pose: Option<Pose>,
    map: OccupancyGrid,
    field: LikelihoodField,
    /// Full field rebuild required before the next match (fresh restore).
    field_stale: bool,
    pose_history: VecDeque<(Instant, Pose)>,
    last_scan_time: Option<Instant>,
    /// Odometry trust: degraded widens the search window and zeroes the
    /// translation prior (encoder fault mode). Set from HR_ODOM_TRUST.
    degraded_odometry: bool,
    last_score: f32,
    low_score_streak: u32,
    reloc_confirms: u32,
    mode: SlamMode,
}

impl GridSlam {
    pub fn new() -> Self {
        Self::with_map(OccupancyGrid::new(600, 600, 0.05), Pose { x: 0.0, y: 0.0, theta: 0.0 })
    }

    /// Rebuild from a persisted map and pose (server restart). The field is
    /// derived from the grid, so matching works from the first sweep; the
    /// engine starts in Relocalizing — wide window, map frozen — until
    /// [`RELOC_CONFIRMS`] consecutive sweeps confirm the pose. This replaces
    /// BasicSlam's `update_count = 1` hack and its dead-reckoning warm-up.
    pub fn restore(map: OccupancyGrid, pose: Pose) -> Self {
        Self::with_map(map, pose)
    }

    fn with_map(map: OccupancyGrid, pose: Pose) -> Self {
        let occupied = map.data.iter().filter(|&&v| cell_is_occupied(v)).count();
        let field = LikelihoodField::new(map.width, map.height);
        Self {
            current_pose: pose,
            last_odom_pose: None,
            map,
            field,
            field_stale: true,
            pose_history: VecDeque::new(),
            last_scan_time: None,
            degraded_odometry: std::env::var("HR_ODOM_TRUST").as_deref() == Ok("low"),
            last_score: 0.0,
            low_score_streak: 0,
            reloc_confirms: 0,
            mode: if occupied < BOOTSTRAP_MIN_OCCUPIED {
                SlamMode::Bootstrap
            } else {
                SlamMode::Relocalizing
            },
        }
    }

    pub fn map(&self) -> &OccupancyGrid {
        &self.map
    }

    /// See [`crate::slam::BasicSlam::on_odometry_reset`].
    pub fn on_odometry_reset(&mut self) {
        self.last_odom_pose = None;
        self.pose_history.clear();
    }

    #[cfg(test)]
    pub fn set_degraded_odometry(&mut self, degraded: bool) {
        self.degraded_odometry = degraded;
    }

    fn window(&self) -> &'static SearchWindow {
        if self.degraded_odometry { &WINDOW_DEGRADED } else { &WINDOW_NOMINAL }
    }

    /// Applies the incremental odometry delta to the current pose estimate.
    fn predict(&mut self, odom_pose: &Pose) {
        if let Some(last_odom) = self.last_odom_pose {
            let dx = odom_pose.x - last_odom.x;
            let dy = odom_pose.y - last_odom.y;
            let dt = odom_pose.theta - last_odom.theta;

            if self.degraded_odometry {
                // Encoder fault: translation is garbage, heading is gyro
                // (trustworthy). Zero-motion translation prior; the wider
                // window still covers real inter-sweep motion.
                self.current_pose.theta += dt;
            } else {
                let diff_theta = self.current_pose.theta - last_odom.theta;
                let (sin_diff, cos_diff) = diff_theta.sin_cos();
                self.current_pose.x += dx * cos_diff - dy * sin_diff;
                self.current_pose.y += dx * sin_diff + dy * cos_diff;
                self.current_pose.theta += dt;
            }
            while self.current_pose.theta > std::f32::consts::PI {
                self.current_pose.theta -= 2.0 * std::f32::consts::PI;
            }
            while self.current_pose.theta < -std::f32::consts::PI {
                self.current_pose.theta += 2.0 * std::f32::consts::PI;
            }
        }
        self.last_odom_pose = Some(*odom_pose);
    }

    /// World points of the sweep at hypothesis `pose` (deskewed when a pose
    /// history exists), plus the same points in the robot frame of `pose`
    /// for the matcher.
    fn project(
        &self,
        scan: &[LidarPoint],
        now: Instant,
        scan_duration: Duration,
        pose: &Pose,
        odom_pose: &Pose,
    ) -> (PointSet, PointSet) {
        let world = if self.pose_history.is_empty() {
            cartesian_points(pose, scan)
        } else {
            deskewed_cartesian_points(&self.pose_history, scan, now, scan_duration, pose, odom_pose)
        };
        let (sin_t, cos_t) = pose.theta.sin_cos();
        let robot = world
            .iter()
            .map(|&(wx, wy)| {
                let dx = wx - pose.x;
                let dy = wy - pose.y;
                (dx * cos_t + dy * sin_t, -dx * sin_t + dy * cos_t)
            })
            .collect();
        (world, robot)
    }

    fn refresh_field(&mut self) {
        if self.field_stale {
            self.map.take_dirty_occupancy();
            self.field.rebuild_full(&self.map);
            self.field_stale = false;
        } else if let Some(dirty) = self.map.take_dirty_occupancy() {
            self.field.rebuild_region(&self.map, dirty);
        }
    }

    fn apply_to_map(&mut self, world_points: &[(f32, f32)]) {
        self.map
            .update_from_deskewed_scan((self.current_pose.x, self.current_pose.y), world_points);
        if self.mode == SlamMode::Bootstrap {
            let occupied = self.map.data.iter().filter(|&&v| cell_is_occupied(v)).count();
            if occupied >= BOOTSTRAP_MIN_OCCUPIED {
                self.mode = SlamMode::Tracking;
            }
        }
    }

}

impl Slam for GridSlam {
    fn update(&mut self, scan: &[LidarPoint], odom_pose: &Pose) -> Pose {
        let now = Instant::now();
        let scan_duration = if let Some(last) = self.last_scan_time {
            let diff = now.duration_since(last);
            if diff > Duration::from_millis(50) && diff < Duration::from_millis(500) {
                diff
            } else {
                Duration::from_millis(150)
            }
        } else {
            Duration::from_millis(150)
        };
        self.last_scan_time = Some(now);

        // 1. Motion prior from odometry.
        self.predict(odom_pose);

        // 2. Bootstrap: nothing to match against — paint and wait for structure.
        if self.mode == SlamMode::Bootstrap {
            let (world, _) = self.project(scan, now, scan_duration, &self.current_pose, odom_pose);
            self.apply_to_map(&world);
            self.last_score = 0.0;
            return self.current_pose;
        }

        // 3. Match against the map. Outside normal tracking the window is
        // always the wide one: re-acquisition must not depend on odometry.
        self.refresh_field();
        let (_, robot_points) = self.project(scan, now, scan_duration, &self.current_pose, odom_pose);
        let prediction = self.current_pose;
        let window = match self.mode {
            SlamMode::Tracking => self.window(),
            _ => &WINDOW_DEGRADED,
        };
        let result = correlative_match(&self.field, &robot_points, &prediction, window);
        let score = result.as_ref().map_or(0.0, |m| m.score);
        self.last_score = score;

        // 4. Mode state machine + gated map update (re-projected at the
        // corrected pose). A frozen map is the safety property: a lost robot
        // must never scribble over good structure.
        let mut update_map = false;
        match self.mode {
            SlamMode::Tracking => {
                if let Some(m) = &result {
                    if m.score >= MATCH_ACCEPT_SCORE {
                        self.current_pose = m.pose;
                    }
                }
                if score < DIVERGED_SCORE {
                    self.low_score_streak += 1;
                    if self.low_score_streak >= DIVERGED_AFTER {
                        self.mode = SlamMode::Diverged;
                        log::warn!(
                            "[SLAM] Match quality collapsed (score {:.2} for {} sweeps); map frozen, widening search",
                            score, self.low_score_streak
                        );
                    }
                } else {
                    self.low_score_streak = 0;
                }
                update_map = score >= MAP_UPDATE_MIN_SCORE;
            }
            SlamMode::Diverged => {
                if let Some(m) = &result {
                    if m.score >= RECOVER_SCORE {
                        self.current_pose = m.pose;
                        self.mode = SlamMode::Tracking;
                        self.low_score_streak = 0;
                        update_map = m.score >= MAP_UPDATE_MIN_SCORE;
                        log::info!("[SLAM] Re-acquired pose (score {:.2}); tracking resumed", m.score);
                    }
                }
            }
            SlamMode::Relocalizing => {
                // Map stays frozen until the pose is confirmed, even on a
                // good match — a stale map + wrong pose is the danger case.
                if let Some(m) = &result {
                    if m.score >= RECOVER_SCORE {
                        self.current_pose = m.pose;
                        self.reloc_confirms += 1;
                        if self.reloc_confirms >= RELOC_CONFIRMS {
                            self.mode = SlamMode::Tracking;
                            log::info!(
                                "[SLAM] Relocalized against the loaded map (score {:.2})",
                                m.score
                            );
                        }
                    } else {
                        self.reloc_confirms = 0;
                    }
                }
            }
            SlamMode::Bootstrap => unreachable!("handled above"),
        }

        if update_map {
            let (world, _) = self.project(scan, now, scan_duration, &self.current_pose, odom_pose);
            self.apply_to_map(&world);
        }

        self.current_pose
    }

    fn add_odom_pose(&mut self, pose: Pose, timestamp: Instant) {
        self.pose_history.push_back((timestamp, pose));
        while !self.pose_history.is_empty()
            && timestamp.duration_since(self.pose_history[0].0) > Duration::from_secs(2)
        {
            self.pose_history.pop_front();
        }
    }

    fn get_map_data(&self) -> (usize, usize, &[i16]) {
        (self.map.width, self.map.height, &self.map.data)
    }

    fn save_map(&self, filename: &str) -> std::io::Result<()> {
        self.map.save_pgm(filename)
    }

    fn get_frontiers(&self) -> Vec<crate::mapping::Frontier> {
        self.map.find_frontiers()
    }

    fn plan_path(&self, goal_x: f32, goal_y: f32) -> Option<Vec<(f32, f32)>> {
        crate::pathfinding::plan_path(&self.map, self.current_pose.x, self.current_pose.y, goal_x, goal_y)
    }

    fn health(&self) -> SlamHealth {
        SlamHealth {
            last_score: self.last_score,
            mode: self.mode,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slam::BasicSlam;
    use crate::test_fixtures::{
        corrupt_odometry, sweep_at, Lcg, OdomCorruption, ScanConfig, SyntheticWorld, Trajectory,
    };

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

    /// Drives a SLAM engine along the fixture loop with corrupted odometry;
    /// returns the final positional error against ground truth.
    fn loop_final_error(slam: &mut dyn Slam, subsample: usize) -> f32 {
        let world = SyntheticWorld::test_room(5.0);
        let traj = Trajectory::p2_loop(5.0);
        let odom = corrupt_odometry(
            &traj.poses,
            &OdomCorruption { heading_drift_rad_per_m: 0.02, scale: 1.04, idle_creep_m: 0.0 },
        );
        let mut rng = Lcg(21);
        let cfg = ScanConfig {
            gap_sectors: vec![(100.0, 130.0)],
            range_noise_m: 0.01,
            ..ScanConfig::default()
        };
        let mut truth_last = traj.poses[0];
        let mut estimate = traj.poses[0];
        for i in (0..traj.poses.len()).step_by(subsample) {
            let sweep = sweep_at(&world, &traj.poses[i], &cfg, &mut rng);
            estimate = slam.update(&sweep, &odom[i]);
            truth_last = traj.poses[i];
        }
        (estimate.x - truth_last.x).hypot(estimate.y - truth_last.y)
    }

    #[test]
    fn beats_basicslam_on_the_fixture_loop_with_bad_odometry() {
        // step_by(2) = one sweep per ~10cm of motion — still 2x sparser than
        // the real 5Hz cadence, without making the test take half a minute.
        let mut grid = GridSlam::new();
        let grid_err = loop_final_error(&mut grid, 2);
        let mut basic = BasicSlam::new();
        let basic_err = loop_final_error(&mut basic, 2);

        assert!(
            grid_err < basic_err,
            "GridSlam ({:.3}m) must beat BasicSlam ({:.3}m) on revisit",
            grid_err,
            basic_err
        );
        // Bound under HARSH stress (2x sparser sweeps than the real 5Hz
        // cadence, 30 deg gap fan, noise, 2%/m heading drift + 4% scale):
        // the remaining driver is map-smear feedback, measured ~0.28m here.
        // The authoritative accuracy bars are the sim-benchmark thresholds
        // (design doc 5.4), evaluated at real cadence.
        assert!(grid_err < 0.30, "GridSlam revisit error {:.3}m too large", grid_err);
    }

    #[test]
    fn bootstrap_paints_then_tracks_by_third_sweep() {
        let world = SyntheticWorld::test_room(5.0);
        let pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let mut rng = Lcg(2);
        let mut slam = GridSlam::new();
        assert_eq!(slam.health().mode, SlamMode::Bootstrap);

        for _ in 0..3 {
            let sweep = sweep_at(&world, &pose, &ScanConfig::default(), &mut rng);
            slam.update(&sweep, &pose);
        }
        assert_eq!(slam.health().mode, SlamMode::Tracking);
        assert!(slam.health().last_score > 0.5, "score {} after settling", slam.health().last_score);
    }

    #[test]
    fn low_score_sweeps_leave_the_grid_untouched() {
        let world = SyntheticWorld::test_room(5.0);
        let pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let mut rng = Lcg(4);
        let mut slam = GridSlam::new();
        for (jx, jy) in [(0.0, 0.0), (0.05, -0.04), (-0.04, 0.06)] {
            let v = Pose { x: pose.x + jx, y: pose.y + jy, theta: pose.theta };
            let sweep = sweep_at(&world, &v, &ScanConfig::default(), &mut rng);
            slam.update(&sweep, &v);
        }
        assert_eq!(slam.health().mode, SlamMode::Tracking);
        let snapshot = slam.map.data.clone();

        // A sweep from a completely different place (2.5m off — outside any
        // window) while odometry claims no motion: the matcher cannot align
        // it, the score collapses, and the map must not change.
        let kidnapped = Pose { x: 3.5, y: -3.5, theta: 1.2 };
        let sweep = sweep_at(&world, &kidnapped, &ScanConfig::default(), &mut rng);
        slam.update(&sweep, &pose);

        assert!(
            slam.health().last_score < MAP_UPDATE_MIN_SCORE,
            "mismatched sweep unexpectedly scored {:.2}",
            slam.health().last_score
        );
        assert_eq!(slam.map.data, snapshot, "gate must freeze the grid on low score");
    }

    #[test]
    fn restore_matches_against_the_loaded_map_immediately() {
        // The SLAM frame is rooted where the FIRST session started (the
        // engine's first update applies no odometry delta), and the saved
        // pose lives in that frame — the test must mirror that, like
        // house_map.bin does.
        let world = SyntheticWorld::test_room(5.0);
        let mut rng = Lcg(8);

        // Session one: build a map; keep the engine's own final estimate.
        let mut first = GridSlam::new();
        let mut saved_pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        for (jx, jy) in [(0.0, 0.0), (0.05, -0.04), (-0.04, 0.06)] {
            let v = Pose { x: jx, y: jy, theta: 0.0 };
            let sweep = sweep_at(&world, &v, &ScanConfig::default(), &mut rng);
            saved_pose = first.update(&sweep, &v);
        }

        // "Restart": engine from the persisted grid+pose; the robot actually
        // sits displaced from where it was saved (moved while powered off).
        let mut restored = GridSlam::restore(first.map.clone(), saved_pose);
        restored.on_odometry_reset();
        assert_eq!(
            restored.health().mode,
            SlamMode::Relocalizing,
            "loaded structure => wide-window confirmation, not bootstrap"
        );
        let map_snapshot = restored.map.data.clone();

        let true_pose = Pose {
            x: saved_pose.x + 0.15,
            y: saved_pose.y - 0.10,
            theta: saved_pose.theta + 0.05,
        };
        // Odometry restarted at zero; first update has no delta to apply.
        let odom_zero = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let sweep = sweep_at(&world, &true_pose, &ScanConfig::default(), &mut rng);
        let corrected = restored.update(&sweep, &odom_zero);
        let err = (corrected.x - true_pose.x).hypot(corrected.y - true_pose.y);
        assert!(err <= 0.06, "restored engine off by {:.3}m on first sweep", err);
        assert!(wrap(corrected.theta - true_pose.theta).abs() < 0.02);
        assert_eq!(restored.health().mode, SlamMode::Relocalizing, "one confirm is not enough");
        assert_eq!(restored.map.data, map_snapshot, "map frozen while relocalizing");

        let sweep = sweep_at(&world, &true_pose, &ScanConfig::default(), &mut rng);
        restored.update(&sweep, &odom_zero);
        assert_eq!(restored.health().mode, SlamMode::Tracking, "two confirms => tracking");
    }

    #[test]
    fn restore_against_a_wrong_map_never_confirms_or_writes() {
        let world = SyntheticWorld::test_room(5.0);
        let pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let mut rng = Lcg(13);
        let mut slam = GridSlam::new();
        for (jx, jy) in [(0.0, 0.0), (0.05, -0.04), (-0.04, 0.06)] {
            let v = Pose { x: pose.x + jx, y: pose.y + jy, theta: pose.theta };
            let sweep = sweep_at(&world, &v, &ScanConfig::default(), &mut rng);
            slam.update(&sweep, &v);
        }

        // Restore that map, but the robot now lives in a different world.
        let other = SyntheticWorld::arena(2.0);
        let mut restored = GridSlam::restore(slam.map.clone(), pose);
        let snapshot = restored.map.data.clone();
        for _ in 0..5 {
            let sweep = sweep_at(&other, &pose, &ScanConfig::default(), &mut rng);
            restored.update(&sweep, &pose);
        }
        assert_eq!(restored.health().mode, SlamMode::Relocalizing, "wrong map must not confirm");
        assert_eq!(restored.map.data, snapshot, "wrong map must never be written");
    }

    #[test]
    fn kidnap_diverges_then_recovers_within_the_wide_window() {
        let world = SyntheticWorld::test_room(5.0);
        let pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let mut rng = Lcg(17);
        let mut slam = GridSlam::new();
        for (jx, jy) in [(0.0, 0.0), (0.05, -0.04), (-0.04, 0.06), (0.03, 0.05)] {
            let v = Pose { x: pose.x + jx, y: pose.y + jy, theta: pose.theta };
            let sweep = sweep_at(&world, &v, &ScanConfig::default(), &mut rng);
            slam.update(&sweep, &v);
        }
        assert_eq!(slam.health().mode, SlamMode::Tracking);
        let snapshot = slam.map.data.clone();

        // Far kidnap (2.5m): no window covers it — the score collapses,
        // tracking degrades to Diverged after the streak, the map freezes,
        // and it must NOT recover onto wrong structure while displaced.
        let moved = Pose { x: 3.5, y: -3.5, theta: 1.2 };
        for _ in 0..DIVERGED_AFTER + 2 {
            let sweep = sweep_at(&world, &moved, &ScanConfig::default(), &mut rng);
            slam.update(&sweep, &pose);
        }
        assert_eq!(slam.health().mode, SlamMode::Diverged, "far kidnap must diverge");
        assert_eq!(slam.map.data, snapshot, "map must stay frozen while lost");

        // The robot is put back near where it was believed to be (within the
        // wide window): recovery to Tracking.
        let returned = Pose { x: 0.3, y: 0.1, theta: 0.1 };
        let mut recovered = false;
        for _ in 0..3 {
            let sweep = sweep_at(&world, &returned, &ScanConfig::default(), &mut rng);
            let est = slam.update(&sweep, &pose);
            if slam.health().mode == SlamMode::Tracking {
                let err = (est.x - returned.x).hypot(est.y - returned.y);
                assert!(err < 0.08, "recovered onto a wrong pose: {:.3}m off", err);
                recovered = true;
                break;
            }
        }
        assert!(recovered, "return within the wide window must re-acquire");
    }

    #[test]
    fn degraded_odometry_mode_tracks_with_garbage_translation() {
        let world = SyntheticWorld::test_room(5.0);
        // Small patrol at REAL sweep density (every 2nd 5Hz pose = ~10cm of
        // motion per sweep); the full p2 loop at 4x subsampling starves the
        // matcher of overlap, which no real 5Hz robot does.
        let traj = Trajectory::patrol(
            Pose { x: 0.0, y: 0.0, theta: 0.0 },
            &[(-2.5, 0.0), (-2.5, 2.5), (0.0, 2.5), (0.0, 0.0)],
            0.25,
            0.8,
            5.0,
        );
        // Fault-like corruption: scale way off plus idle creep.
        let odom = corrupt_odometry(
            &traj.poses,
            &OdomCorruption { heading_drift_rad_per_m: 0.0, scale: 1.5, idle_creep_m: 0.004 },
        );
        let mut rng = Lcg(23);
        let mut slam = GridSlam::new();
        slam.set_degraded_odometry(true);

        let mut estimate = traj.poses[0];
        let mut truth = traj.poses[0];
        for i in (0..traj.poses.len()).step_by(2) {
            let sweep = sweep_at(&world, &traj.poses[i], &ScanConfig::default(), &mut rng);
            estimate = slam.update(&sweep, &odom[i]);
            truth = traj.poses[i];
        }
        assert_eq!(slam.health().mode, SlamMode::Tracking, "must not diverge on a clean patrol");
        let err = (estimate.x - truth.x).hypot(estimate.y - truth.y);
        assert!(
            err < 0.20,
            "degraded mode must track through garbage odometry, final error {:.3}m",
            err
        );
    }

    #[test]
    fn update_stays_within_realtime_budget_in_debug() {
        let world = SyntheticWorld::test_room(5.0);
        let pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let mut rng = Lcg(6);
        let mut slam = GridSlam::new();
        for (jx, jy) in [(0.0, 0.0), (0.05, -0.04), (-0.04, 0.06)] {
            let v = Pose { x: pose.x + jx, y: pose.y + jy, theta: pose.theta };
            let sweep = sweep_at(&world, &v, &ScanConfig::default(), &mut rng);
            slam.update(&sweep, &v);
        }
        let sweep = sweep_at(&world, &pose, &ScanConfig::default(), &mut rng);
        let start = std::time::Instant::now();
        slam.update(&sweep, &pose);
        let elapsed = start.elapsed();
        assert!(elapsed.as_millis() < 60, "update took {:?} (debug budget 60ms)", elapsed);
    }
}
