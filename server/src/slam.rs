use crate::homerobot::LidarPoint;
use crate::odometry::Pose;
use crate::mapping::OccupancyGrid;

/// The core trait for SLAM algorithms. 
pub trait Slam {
    /// Feed a new LiDAR scan and current odometry estimate.
    /// Returns the corrected Pose.
    fn update(&mut self, scan: &[LidarPoint], odom_pose: &Pose) -> Pose;

    /// Get the current probability map data.
    fn get_map_data(&self) -> (usize, usize, &[i16]);

    /// Save the map to a file.
    fn save_map(&self, filename: &str) -> std::io::Result<()>;

    /// Find frontiers for exploration.
    fn get_frontiers(&self) -> Vec<crate::mapping::Frontier>;

    /// Plan a path to a goal.
    fn plan_path(&self, goal_x: f32, goal_y: f32) -> Option<Vec<(f32, f32)>>;
}

/// A basic incremental SLAM implementation.
/// Uses Hill Climbing Scan-to-Map matching.
pub struct BasicSlam {
    current_pose: Pose,
    last_odom_pose: Option<Pose>,
    map: OccupancyGrid,
    update_count: usize,
    reference_cloud: Vec<(f32, f32)>,
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
        }
    }

    fn get_cartesian_points(pose: &Pose, scan: &[LidarPoint]) -> Vec<(f32, f32)> {
        let mut points = Vec::new();
        for p in scan {
            if p.distance_mm < 200.0 || p.distance_mm > 5000.0 { continue; }
            let angle_rad = (p.angle_deg as f32).to_radians();
            let global_angle = pose.theta + angle_rad;
            let dist_m = p.distance_mm / 1000.0;
            let x = pose.x + dist_m * global_angle.cos();
            let y = pose.y + dist_m * global_angle.sin();
            points.push((x, y));
        }
        points
    }

    /// Refine pose using K-Nearest Neighbor Iterative Closest Point (ICP)
    fn icp_match(&self, initial_pose: &Pose, scan: &[LidarPoint]) -> Pose {
        if self.reference_cloud.is_empty() {
            return *initial_pose;
        }
        let mut current_pose = *initial_pose;
        for _iter in 0..15 {
            let q_points = Self::get_cartesian_points(&current_pose, scan);
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

            let old_x = current_pose.x;
            let old_y = current_pose.y;
            current_pose.x = old_x * cos_dt - old_y * sin_dt + dx;
            current_pose.y = old_x * sin_dt + old_y * cos_dt + dy;
            current_pose.theta += dt;
            if dx.abs() < 0.0001 && dy.abs() < 0.0001 && dt.abs() < 0.0001 { break; }
        }
        current_pose
    }
}

impl Slam for BasicSlam {
    fn update(&mut self, scan: &[LidarPoint], odom_pose: &Pose) -> Pose {
        self.update_count += 1;

        // 1. Calculate Odom Delta
        if let Some(last_odom) = self.last_odom_pose {
            let dx = odom_pose.x - last_odom.x;
            let dy = odom_pose.y - last_odom.y;
            let dt = odom_pose.theta - last_odom.theta;

            // Simple movement model: add delta to current pose
            // This is "Dead Reckoning" as a starting point for the matcher
            self.current_pose.x += dx;
            self.current_pose.y += dy;
            self.current_pose.theta += dt;
        } else {
            self.current_pose = *odom_pose;
        }
        self.last_odom_pose = Some(*odom_pose);

        // 2. Localization Refinement (Scan-to-Map Matching)
        // Only refine if we have some map data
        if self.update_count > 1 {
            self.current_pose = self.icp_match(&self.current_pose, scan);
        }

        // 3. Mapping Update
        self.map.update_from_scan(&self.current_pose, scan);

        // 4. Update Reference Point Cloud
        let q_points = BasicSlam::get_cartesian_points(&self.current_pose, scan);
        for p in q_points {
            self.reference_cloud.push(p);
        }
        if self.reference_cloud.len() > 1000 {
            let excess = self.reference_cloud.len() - 1000;
            self.reference_cloud.drain(0..excess);
        }

        self.current_pose
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
