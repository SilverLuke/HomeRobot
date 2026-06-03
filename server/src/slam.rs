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
}

impl BasicSlam {
    pub fn new() -> Self {
        Self {
            current_pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            last_odom_pose: None,
            // 30m x 30m map at 5cm resolution (600x600 pixels)
            map: OccupancyGrid::new(600, 600, 0.05),
            update_count: 0,
        }
    }

    /// Calculate how well a pose matches the current map
    fn score_pose(&self, pose: &Pose, scan: &[LidarPoint]) -> f32 {
        let mut score = 0.0;
        for p in scan {
            if p.distance_mm < 200.0 || p.distance_mm > 5000.0 { continue; }
            
            let angle_rad = (p.angle_deg as f32).to_radians();
            let total_angle = pose.theta + angle_rad;
            
            let ox = pose.x + (p.distance_mm / 1000.0) * total_angle.cos();
            let oy = pose.y + (p.distance_mm / 1000.0) * total_angle.sin();
            
            if self.map.get_cell(ox, oy) == 100 {
                score += 1.0;
            } else if self.map.get_cell(ox, oy) == 0 {
                score -= 0.5; // Penalty for hitting free space
            }
        }
        score
    }

    /// Refine the pose using a simple hill-climbing search
    fn refine_pose(&self, start_pose: Pose, scan: &[LidarPoint]) -> Pose {
        let mut best_pose = start_pose;
        let mut best_score = self.score_pose(&best_pose, scan);

        let steps_m = [0.01, 0.02, 0.05]; // 1cm, 2cm, 5cm
        let steps_rad = [0.01, 0.02, 0.05]; // ~0.5, 1, 3 degrees

        let mut improved = true;
        let mut iterations = 0;

        while improved && iterations < 20 {
            improved = false;
            iterations += 1;

            for &dx in &[-1.0, 0.0, 1.0] {
                for &dy in &[-1.0, 0.0, 1.0] {
                    for &dt in &[-1.0, 0.0, 1.0] {
                        if dx == 0.0 && dy == 0.0 && dt == 0.0 { continue; }

                        for i in 0..steps_m.len() {
                            let test_pose = Pose {
                                x: best_pose.x + dx * steps_m[i],
                                y: best_pose.y + dy * steps_m[i],
                                theta: best_pose.theta + dt * steps_rad[i],
                            };

                            let score = self.score_pose(&test_pose, scan);
                            if score > best_score {
                                best_score = score;
                                best_pose = test_pose;
                                improved = true;
                            }
                        }
                    }
                }
            }
        }
        best_pose
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
        // Only refine if we have some map data (after first few scans)
        if self.update_count > 5 {
            self.current_pose = self.refine_pose(self.current_pose, scan);
        }

        // 3. Mapping Update
        self.map.update_from_scan(&self.current_pose, scan);

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
