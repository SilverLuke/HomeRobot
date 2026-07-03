use crate::homerobot::LidarPoint;
use crate::odometry::Pose;
use crate::mapping::OccupancyGrid;
use std::collections::VecDeque;
use std::time::{Instant, Duration};

/// Operating state of a SLAM engine, surfaced for diagnostics and the
/// pose-trace CSV (`mode` column).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SlamMode {
    /// Map has no structure yet; sweeps are applied at the odometry pose.
    Bootstrap,
    /// Normal scan-to-map tracking.
    Tracking,
    /// Re-acquiring the pose against a loaded map (after restart).
    Relocalizing,
    /// Match quality collapsed; map updates frozen.
    Diverged,
}

impl SlamMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            SlamMode::Bootstrap => "bootstrap",
            SlamMode::Tracking => "tracking",
            SlamMode::Relocalizing => "relocalizing",
            SlamMode::Diverged => "diverged",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SlamHealth {
    /// Most recent per-sweep match score in [0, 1].
    pub last_score: f32,
    pub mode: SlamMode,
}

/// The core trait for SLAM algorithms.
pub trait Slam {
    /// Feed a new LiDAR scan and current odometry estimate.
    /// Returns the corrected Pose.
    fn update(&mut self, scan: &[LidarPoint], odom_pose: &Pose) -> Pose;

    /// Like [`update`], anchored to the sweep's ARRIVAL time. The session
    /// event loop can lag the wire (replan stalls); deskewing against
    /// processing time warps the cloud. Engines that deskew must override
    /// this; the default ignores the timestamp (fine for stubs).
    fn update_at(&mut self, scan: &[LidarPoint], odom_pose: &Pose, _scan_end_time: Instant) -> Pose {
        self.update(scan, odom_pose)
    }

    /// Record a high-frequency odometry pose with timestamp.
    fn add_odom_pose(&mut self, pose: Pose, timestamp: Instant);

    /// Get the current probability map data.
    fn get_map_data(&self) -> (usize, usize, &[i16]);

    /// Save the map to a file.
    fn save_map(&self, filename: &str) -> std::io::Result<()>;

    /// Find frontiers for exploration.
    fn get_frontiers(&self) -> Vec<crate::mapping::Frontier>;

    /// Plan a path to a goal.
    fn plan_path(&self, goal_x: f32, goal_y: f32) -> Option<Vec<(f32, f32)>>;

    /// Match quality / operating state. Engines without a score concept
    /// (BasicSlam) report the default.
    fn health(&self) -> SlamHealth {
        SlamHealth { last_score: 0.0, mode: SlamMode::Tracking }
    }
}

/// A basic incremental SLAM implementation.
/// Uses Hill Climbing Scan-to-Map matching.
pub struct BasicSlam {
    current_pose: Pose,
    last_odom_pose: Option<Pose>,
    map: OccupancyGrid,
    update_count: usize,
    reference_cloud: Vec<(f32, f32)>,
    pose_history: VecDeque<(Instant, Pose)>,
    last_scan_time: Option<Instant>,
}

impl BasicSlam {
    pub fn new() -> Self {
        Self {
            current_pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            last_odom_pose: None,
            // 30m x 30m map at 5cm resolution (600x600 pixels)
            map: OccupancyGrid::new(600, 600, 0.05),
            update_count: 0,
            reference_cloud: Vec::new(),
            pose_history: VecDeque::new(),
            last_scan_time: None,
        }
    }

    fn get_cartesian_points(pose: &Pose, scan: &[LidarPoint]) -> Vec<(f32, f32)> {
        cartesian_points(pose, scan)
    }

    #[cfg(test)]
    fn interpolate_pose_at_time(&self, time: Instant) -> Pose {
        interpolate_pose_at(&self.pose_history, time)
    }

    fn get_deskewed_cartesian_points(
        &self,
        scan: &[LidarPoint],
        scan_end_time: Instant,
        scan_duration: Duration,
        slam_end_pose: &Pose,
        odom_end_pose: &Pose
    ) -> Vec<(f32, f32)> {
        deskewed_cartesian_points(&self.pose_history, scan, scan_end_time, scan_duration, slam_end_pose, odom_end_pose)
    }

    /// Refine pose using K-Nearest Neighbor Iterative Closest Point (ICP)
    fn icp_match(
        &self, 
        initial_pose: &Pose, 
        scan: &[LidarPoint], 
        scan_end_time: Instant, 
        scan_duration: Duration, 
        odom_end_pose: &Pose
    ) -> Pose {
        if self.reference_cloud.is_empty() {
            return *initial_pose;
        }
        let mut current_pose = *initial_pose;
        for _iter in 0..15 {
            let q_points = if self.pose_history.is_empty() {
                Self::get_cartesian_points(&current_pose, scan)
            } else {
                self.get_deskewed_cartesian_points(scan, scan_end_time, scan_duration, &current_pose, odom_end_pose)
            };
            if q_points.is_empty() { break; }
            let mut matched_p = Vec::new();
            let mut matched_q = Vec::new();
            for &q in &q_points {
                let mut min_dist2 = f32::MAX;
                let mut nearest_p = (0.0, 0.0);
                for &p in &self.reference_cloud {
                    let dist2 = (q.0 - p.0).powi(2) + (q.1 - p.1).powi(2);
                    if dist2 < min_dist2 {
                        min_dist2 = dist2;
                        nearest_p = p;
                    }
                }
                if min_dist2 < 0.2_f32.powi(2) { // 20cm outlier rejection
                    matched_q.push(q);
                    matched_p.push(nearest_p);
                }
            }
            if matched_q.len() < 10 { break; }
            let n = matched_q.len() as f32;
            let mut mean_p = (0.0, 0.0);
            let mut mean_q = (0.0, 0.0);
            for i in 0..matched_q.len() {
                mean_p.0 += matched_p[i].0;
                mean_p.1 += matched_p[i].1;
                mean_q.0 += matched_q[i].0;
                mean_q.1 += matched_q[i].1;
            }
            mean_p.0 /= n; mean_p.1 /= n;
            mean_q.0 /= n; mean_q.1 /= n;
            let mut s_xx = 0.0;
            let mut s_xy = 0.0;
            let mut s_yx = 0.0;
            let mut s_yy = 0.0;
            for i in 0..matched_q.len() {
                let p_prime = (matched_p[i].0 - mean_p.0, matched_p[i].1 - mean_p.1);
                let q_prime = (matched_q[i].0 - mean_q.0, matched_q[i].1 - mean_q.1);
                s_xx += p_prime.0 * q_prime.0;
                s_xy += p_prime.0 * q_prime.1;
                s_yx += p_prime.1 * q_prime.0;
                s_yy += p_prime.1 * q_prime.1;
            }
            let dt = (s_yx - s_xy).atan2(s_xx + s_yy);
            let cos_dt = dt.cos();
            let sin_dt = dt.sin();
            let dx = mean_p.0 - (mean_q.0 * cos_dt - mean_q.1 * sin_dt);
            let dy = mean_p.1 - (mean_q.0 * sin_dt + mean_q.1 * cos_dt);
            
            // Add odometry spring penalty to avoid random walk on flat walls
            // We pull dx, dy, dt slightly back towards 0
            let pull = 0.95; 
            let dx = dx * pull;
            let dy = dy * pull;
            let dt = dt * pull;

            let cos_dt_pulled = dt.cos();
            let sin_dt_pulled = dt.sin();

            let old_x = current_pose.x;
            let old_y = current_pose.y;
            current_pose.x = old_x * cos_dt_pulled - old_y * sin_dt_pulled + dx;
            current_pose.y = old_x * sin_dt_pulled + old_y * cos_dt_pulled + dy;
            current_pose.theta += dt;

            // Normalize theta to [-PI, PI]
            while current_pose.theta > std::f32::consts::PI { current_pose.theta -= 2.0 * std::f32::consts::PI; }
            while current_pose.theta < -std::f32::consts::PI { current_pose.theta += 2.0 * std::f32::consts::PI; }

            if dx.abs() < 0.0001 && dy.abs() < 0.0001 && dt.abs() < 0.0001 { break; }
        }
        current_pose
    }

    /// Forget odometry-frame tracking state. Called when a robot session
    /// begins: each session's server-side odometry restarts at zero, so
    /// deltas across the boundary would be garbage. Map and pose survive.
    pub fn on_odometry_reset(&mut self) {
        self.last_odom_pose = None;
        self.pose_history.clear();
    }

    pub fn map(&self) -> &OccupancyGrid {
        &self.map
    }

    /// Rebuild SLAM state from a persisted map and pose (server restart).
    /// The ICP reference cloud is rebuilt from live scans; until then the
    /// pose is dead-reckoned only.
    pub fn restore(map: OccupancyGrid, pose: Pose) -> Self {
        let mut slam = Self::new();
        slam.map = map;
        slam.current_pose = pose;
        slam.update_count = 1;
        slam
    }
}

impl Slam for BasicSlam {
    fn update(&mut self, scan: &[LidarPoint], odom_pose: &Pose) -> Pose {
        self.update_at(scan, odom_pose, Instant::now())
    }

    fn update_at(&mut self, scan: &[LidarPoint], odom_pose: &Pose, now: Instant) -> Pose {
        self.update_count += 1;

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

        // 1. Calculate Odom Delta. With no previous odometry sample (first
        // sweep of a session) there is no delta to apply: keep the current
        // pose — it may carry state from a previous session.
        if let Some(last_odom) = self.last_odom_pose {
            let dx = odom_pose.x - last_odom.x;
            let dy = odom_pose.y - last_odom.y;
            let dt = odom_pose.theta - last_odom.theta;

            // Rotate odometry delta (dx, dy) to align with SLAM pose orientation
            let diff_theta = self.current_pose.theta - last_odom.theta;
            let cos_diff = diff_theta.cos();
            let sin_diff = diff_theta.sin();
            let dx_map = dx * cos_diff - dy * sin_diff;
            let dy_map = dx * sin_diff + dy * cos_diff;

            // Simple movement model: add delta to current pose
            // This is "Dead Reckoning" as a starting point for the matcher
            self.current_pose.x += dx_map;
            self.current_pose.y += dy_map;
            self.current_pose.theta += dt;

            // Normalize theta to [-PI, PI]
            while self.current_pose.theta > std::f32::consts::PI { self.current_pose.theta -= 2.0 * std::f32::consts::PI; }
            while self.current_pose.theta < -std::f32::consts::PI { self.current_pose.theta += 2.0 * std::f32::consts::PI; }
        }
        self.last_odom_pose = Some(*odom_pose);

        // 2. Localization Refinement (Scan-to-Map Matching)
        // Only refine if we have some map data
        if self.update_count > 1 {
            self.current_pose = self.icp_match(&self.current_pose, scan, now, scan_duration, odom_pose);
        }

        // 3. Mapping Update
        let q_points = if self.pose_history.is_empty() {
            Self::get_cartesian_points(&self.current_pose, scan)
        } else {
            self.get_deskewed_cartesian_points(scan, now, scan_duration, &self.current_pose, odom_pose)
        };

        self.map.update_from_deskewed_scan((self.current_pose.x, self.current_pose.y), &q_points);

        // 4. Update Reference Point Cloud
        for p in q_points {
            self.reference_cloud.push(p);
        }
        if self.reference_cloud.len() > 1000 {
            let excess = self.reference_cloud.len() - 1000;
            self.reference_cloud.drain(0..excess);
        }

        self.current_pose
    }

    fn add_odom_pose(&mut self, pose: Pose, timestamp: Instant) {
        self.pose_history.push_back((timestamp, pose));
        while !self.pose_history.is_empty() && timestamp.duration_since(self.pose_history[0].0) > Duration::from_secs(2) {
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
}

// ---- Engine selection (plan T11) ---------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EngineKind {
    Basic,
    Grid,
}

impl EngineKind {
    /// `HR_SLAM=grid` selects the map-anchored engine; anything else (or
    /// unset) keeps the rolling-cloud `BasicSlam` — the default until the
    /// promotion gate (plan T15) flips it.
    pub fn from_env() -> Self {
        match std::env::var("HR_SLAM").as_deref() {
            Ok("grid") => EngineKind::Grid,
            _ => EngineKind::Basic,
        }
    }
}

/// The active SLAM engine. An enum rather than `Box<dyn Slam>` because
/// restore/save need concrete types and there are exactly two variants.
pub enum SlamEngine {
    Basic(BasicSlam),
    Grid(crate::slam_grid::GridSlam),
}

impl SlamEngine {
    pub fn new(kind: EngineKind) -> Self {
        match kind {
            EngineKind::Basic => SlamEngine::Basic(BasicSlam::new()),
            EngineKind::Grid => SlamEngine::Grid(crate::slam_grid::GridSlam::new()),
        }
    }

    pub fn restore(kind: EngineKind, map: crate::mapping::OccupancyGrid, pose: Pose) -> Self {
        match kind {
            EngineKind::Basic => SlamEngine::Basic(BasicSlam::restore(map, pose)),
            EngineKind::Grid => SlamEngine::Grid(crate::slam_grid::GridSlam::restore(map, pose)),
        }
    }

    pub fn map(&self) -> &OccupancyGrid {
        match self {
            SlamEngine::Basic(s) => s.map(),
            SlamEngine::Grid(s) => s.map(),
        }
    }

    pub fn on_odometry_reset(&mut self) {
        match self {
            SlamEngine::Basic(s) => s.on_odometry_reset(),
            SlamEngine::Grid(s) => s.on_odometry_reset(),
        }
    }
}

impl Slam for SlamEngine {
    fn update(&mut self, scan: &[LidarPoint], odom_pose: &Pose) -> Pose {
        match self {
            SlamEngine::Basic(s) => s.update(scan, odom_pose),
            SlamEngine::Grid(s) => s.update(scan, odom_pose),
        }
    }

    fn update_at(&mut self, scan: &[LidarPoint], odom_pose: &Pose, scan_end_time: Instant) -> Pose {
        match self {
            SlamEngine::Basic(s) => s.update_at(scan, odom_pose, scan_end_time),
            SlamEngine::Grid(s) => s.update_at(scan, odom_pose, scan_end_time),
        }
    }

    fn add_odom_pose(&mut self, pose: Pose, timestamp: Instant) {
        match self {
            SlamEngine::Basic(s) => s.add_odom_pose(pose, timestamp),
            SlamEngine::Grid(s) => s.add_odom_pose(pose, timestamp),
        }
    }

    fn get_map_data(&self) -> (usize, usize, &[i16]) {
        match self {
            SlamEngine::Basic(s) => s.get_map_data(),
            SlamEngine::Grid(s) => s.get_map_data(),
        }
    }

    fn save_map(&self, filename: &str) -> std::io::Result<()> {
        match self {
            SlamEngine::Basic(s) => s.save_map(filename),
            SlamEngine::Grid(s) => s.save_map(filename),
        }
    }

    fn get_frontiers(&self) -> Vec<crate::mapping::Frontier> {
        match self {
            SlamEngine::Basic(s) => s.get_frontiers(),
            SlamEngine::Grid(s) => s.get_frontiers(),
        }
    }

    fn plan_path(&self, goal_x: f32, goal_y: f32) -> Option<Vec<(f32, f32)>> {
        match self {
            SlamEngine::Basic(s) => s.plan_path(goal_x, goal_y),
            SlamEngine::Grid(s) => s.plan_path(goal_x, goal_y),
        }
    }

    fn health(&self) -> SlamHealth {
        match self {
            SlamEngine::Basic(s) => s.health(),
            SlamEngine::Grid(s) => s.health(),
        }
    }
}

// ---- Shared projection / deskewing helpers (used by BasicSlam and GridSlam) ----

/// Projects a scan into world coordinates at a single pose (no deskewing).
/// Points outside the 0.2–8m band are dropped.
pub(crate) fn cartesian_points(pose: &Pose, scan: &[LidarPoint]) -> Vec<(f32, f32)> {
    let mut points = Vec::new();
    for p in scan {
        if p.distance_mm < 200.0 || p.distance_mm > 8000.0 { continue; }
        let angle_rad = p.angle_deg.to_radians();
        let global_angle = pose.theta - angle_rad;
        let dist_m = p.distance_mm / 1000.0;
        points.push((pose.x + dist_m * global_angle.cos(), pose.y + dist_m * global_angle.sin()));
    }
    points
}

/// Linear interpolation over a timestamped pose history (angle wrap-aware).
pub(crate) fn interpolate_pose_at(history: &VecDeque<(Instant, Pose)>, time: Instant) -> Pose {
    if history.is_empty() {
        return Pose { x: 0.0, y: 0.0, theta: 0.0 };
    }
    if time <= history[0].0 {
        return history[0].1;
    }
    if time >= history[history.len() - 1].0 {
        return history[history.len() - 1].1;
    }

    // Find the bounding elements
    for i in 0..history.len() - 1 {
        let (t1, p1) = &history[i];
        let (t2, p2) = &history[i + 1];
        if time >= *t1 && time <= *t2 {
            let denom = t2.duration_since(*t1).as_secs_f32();
            let t = if denom > 0.0 {
                time.duration_since(*t1).as_secs_f32() / denom
            } else {
                0.0
            };

            let x = p1.x + (p2.x - p1.x) * t;
            let y = p1.y + (p2.y - p1.y) * t;

            // Angle interpolation with wrap-around
            let mut diff = p2.theta - p1.theta;
            while diff > std::f32::consts::PI { diff -= 2.0 * std::f32::consts::PI; }
            while diff < -std::f32::consts::PI { diff += 2.0 * std::f32::consts::PI; }
            let theta = p1.theta + diff * t;

            return Pose { x, y, theta };
        }
    }

    history[history.len() - 1].1
}

/// Motion-compensated projection: each point is projected from the
/// interpolated pose at its (inferred) capture time, expressed in the SLAM
/// frame via the end-of-sweep SLAM/odometry pose pair.
/// See docs/lidar_deskewing.md.
pub(crate) fn deskewed_cartesian_points(
    history: &VecDeque<(Instant, Pose)>,
    scan: &[LidarPoint],
    scan_end_time: Instant,
    scan_duration: Duration,
    slam_end_pose: &Pose,
    odom_end_pose: &Pose,
) -> Vec<(f32, f32)> {
    let mut points = Vec::new();
    let n = scan.len();
    if n == 0 { return points; }

    let start_time = scan_end_time.checked_sub(scan_duration).unwrap_or(scan_end_time);

    for (i, p) in scan.iter().enumerate() {
        if p.distance_mm < 200.0 || p.distance_mm > 8000.0 { continue; }

        // Infer timestamp of this point
        let t_offset = (i as f32 / n as f32) * scan_duration.as_secs_f32();
        let p_time = start_time + Duration::from_secs_f32(t_offset);

        // Interpolate odometry pose at p_time
        let odom_i = interpolate_pose_at(history, p_time);

        // Relate it back to the SLAM frame using slam_end_pose and odom_end_pose
        let dx = odom_i.x - odom_end_pose.x;
        let dy = odom_i.y - odom_end_pose.y;
        let dt = odom_i.theta - odom_end_pose.theta;

        let diff_theta = slam_end_pose.theta - odom_end_pose.theta;
        let cos_diff = diff_theta.cos();
        let sin_diff = diff_theta.sin();

        let robot_x = slam_end_pose.x + dx * cos_diff - dy * sin_diff;
        let robot_y = slam_end_pose.y + dx * sin_diff + dy * cos_diff;
        let robot_theta = slam_end_pose.theta + dt;

        // Project the point
        let angle_rad = p.angle_deg.to_radians();
        let global_angle = robot_theta - angle_rad;
        let dist_m = p.distance_mm / 1000.0;
        points.push((robot_x + dist_m * global_angle.cos(), robot_y + dist_m * global_angle.sin()));
    }
    points
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Instant, Duration};

    #[test]
    fn test_pose_history_interpolation() {
        let mut slam = BasicSlam::new();
        let t0 = Instant::now();
        
        // Populate trajectory
        slam.add_odom_pose(Pose { x: 0.0, y: 0.0, theta: 0.0 }, t0);
        slam.add_odom_pose(Pose { x: 1.0, y: 2.0, theta: 1.0 }, t0 + Duration::from_millis(100));
        slam.add_odom_pose(Pose { x: 2.0, y: 4.0, theta: 2.0 }, t0 + Duration::from_millis(200));

        // Test boundary limits
        let p_early = slam.interpolate_pose_at_time(t0 - Duration::from_millis(50));
        assert_eq!(p_early.x, 0.0);
        assert_eq!(p_early.y, 0.0);

        let p_late = slam.interpolate_pose_at_time(t0 + Duration::from_millis(250));
        assert_eq!(p_late.x, 2.0);
        assert_eq!(p_late.y, 4.0);

        // Test mid-point interpolation
        let p_mid = slam.interpolate_pose_at_time(t0 + Duration::from_millis(50));
        assert_eq!(p_mid.x, 0.5);
        assert_eq!(p_mid.y, 1.0);
        assert!((p_mid.theta - 0.5).abs() < 1e-5);
    }

    #[test]
    fn test_pose_interpolation_wrap_around() {
        let mut slam = BasicSlam::new();
        let t0 = Instant::now();
        
        // Wrap around CCW: from PI - 0.1 to -PI + 0.1
        slam.add_odom_pose(Pose { x: 0.0, y: 0.0, theta: std::f32::consts::PI - 0.1 }, t0);
        slam.add_odom_pose(Pose { x: 0.0, y: 0.0, theta: -std::f32::consts::PI + 0.1 }, t0 + Duration::from_millis(100));

        // Mid point should be exactly PI / -PI
        let p_mid = slam.interpolate_pose_at_time(t0 + Duration::from_millis(50));
        let expected_theta = std::f32::consts::PI;
        let mut diff = p_mid.theta - expected_theta;
        while diff > std::f32::consts::PI { diff -= 2.0 * std::f32::consts::PI; }
        while diff < -std::f32::consts::PI { diff += 2.0 * std::f32::consts::PI; }
        assert!(diff.abs() < 1e-4);
    }
}
