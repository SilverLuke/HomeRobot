#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

pub struct Odometry {
    pub pose: Pose,
    last_left_ticks: i32,
    last_right_ticks: i32,
    ticks_per_meter: f32,
    wheel_base: f32,
    first_update: bool,
}

impl Odometry {
    pub fn new() -> Self {
        Self {
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            last_left_ticks: 0,
            last_right_ticks: 0,
            ticks_per_meter: 5780.0, // Based on Gazebo bridge
            wheel_base: 0.26,        // 26cm
            first_update: true,
        }
    }

    pub fn update(&mut self, left_ticks: i32, right_ticks: i32) {
        if self.first_update {
            self.last_left_ticks = left_ticks;
            self.last_right_ticks = right_ticks;
            self.first_update = false;
            return;
        }

        let d_left_ticks = left_ticks - self.last_left_ticks;
        let d_right_ticks = right_ticks - self.last_right_ticks;

        let d_left_m = d_left_ticks as f32 / self.ticks_per_meter;
        let d_right_m = d_right_ticks as f32 / self.ticks_per_meter;

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

        self.last_left_ticks = left_ticks;
        self.last_right_ticks = right_ticks;
    }
}
