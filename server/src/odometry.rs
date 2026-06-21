#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

pub struct Odometry {
    pub pose: Pose,
    pub cumulative_left: i32,
    pub cumulative_right: i32,
    ticks_per_meter: f32,
    wheel_base: f32,
}

impl Odometry {
    pub fn new() -> Self {
        Self {
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            cumulative_left: 0,
            cumulative_right: 0,
            ticks_per_meter: 1736.2, // 360 ticks / (2 * pi * 0.033m)
            wheel_base: 0.26,        // 26cm
        }
    }

    pub fn update(&mut self, left_delta: i32, right_delta: i32) {
        self.cumulative_left += left_delta;
        self.cumulative_right += right_delta;

        let d_left_m = left_delta as f32 / self.ticks_per_meter;
        let d_right_m = right_delta as f32 / self.ticks_per_meter;

        let d_center = (d_left_m + d_right_m) / 2.0;
        let d_theta = (d_right_m - d_left_m) / self.wheel_base;

        // Update pose using mid-point orientation for better accuracy
        let avg_theta = self.pose.theta + (d_theta / 2.0);
        self.pose.x += d_center * avg_theta.cos();
        self.pose.y += d_center * avg_theta.sin();
        self.pose.theta += d_theta;

        // Normalize theta to [-PI, PI]
        while self.pose.theta > std::f32::consts::PI { self.pose.theta -= 2.0 * std::f32::consts::PI; }
        while self.pose.theta < -std::f32::consts::PI { self.pose.theta += 2.0 * std::f32::consts::PI; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_odometry_update_delta() {
        let mut odom = Odometry::new();
        assert_eq!(odom.cumulative_left, 0);
        assert_eq!(odom.cumulative_right, 0);
        assert_eq!(odom.pose.x, 0.0);

        // Move forward slightly
        odom.update(1736, 1736); // Approx 1 meter forward
        assert_eq!(odom.cumulative_left, 1736);
        assert_eq!(odom.cumulative_right, 1736);
        assert!((odom.pose.x - 1.0).abs() < 0.01);
        assert_eq!(odom.pose.y, 0.0);
        assert_eq!(odom.pose.theta, 0.0);

        // Turn in place
        odom.update(-425, 425);
        assert_eq!(odom.cumulative_left, 1311);
        assert_eq!(odom.cumulative_right, 2161);
        assert!(odom.pose.theta > 0.0);
    }
}
