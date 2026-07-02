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

    // Sensor Fusion & Calibration State
    last_timestamp_ms: Option<u32>,
    last_gyro_z: f32,
    pub gyro_bias: f32,
    pub stationary_samples: u32,
    stationary_gyro_sum: f32,
    last_encoder_activity_ms: u32,
    has_imu: bool,
}

impl Odometry {
    pub fn new() -> Self {
        Self {
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            cumulative_left: 0,
            cumulative_right: 0,
            ticks_per_meter: 1736.2, // 360 ticks / (2 * pi * 0.033m)
            wheel_base: 0.26,        // 26cm

            last_timestamp_ms: None,
            last_gyro_z: 0.0,
            gyro_bias: 0.0,
            stationary_samples: 0,
            stationary_gyro_sum: 0.0,
            last_encoder_activity_ms: 0,
            has_imu: false,
        }
    }

    pub fn update_sizes(&mut self, wheel_diameter_mm: f32, wheel_track_mm: f32, encoder_ticks_per_rev: u32) {
        let diameter_m = wheel_diameter_mm / 1000.0;
        let track_m = wheel_track_mm / 1000.0;
        
        self.wheel_base = track_m;
        self.ticks_per_meter = (encoder_ticks_per_rev as f32) / (std::f32::consts::PI * diameter_m);
        log::info!(
            "[ODOMETRY] Dynamic configuration applied: wheel_base = {:.4} m, ticks_per_meter = {:.2}",
            self.wheel_base, self.ticks_per_meter
        );
    }

    /// Feed new gyroscope reading from local IMU.
    pub fn update_imu(&mut self, gyro_z: f32, timestamp_ms: u32) {
        self.last_gyro_z = gyro_z;
        self.has_imu = true;

        // Perform zero-velocity calibration when stationary (no encoder delta).
        // We calibrate using the first 30 stationary samples.
        if self.stationary_samples < 30 {
            let time_since_encoder_activity = match self.last_timestamp_ms {
                Some(_) => {
                    if timestamp_ms >= self.last_encoder_activity_ms {
                        timestamp_ms - self.last_encoder_activity_ms
                    } else {
                        0
                    }
                }
                None => 0,
            };

            // If we have been stationary for at least 500ms or just starting up
            if self.last_encoder_activity_ms == 0 || time_since_encoder_activity > 500 {
                self.stationary_gyro_sum += gyro_z;
                self.stationary_samples += 1;
                if self.stationary_samples == 30 {
                    self.gyro_bias = self.stationary_gyro_sum / 30.0;
                    log::info!("[ODOMETRY] Gyro calibration completed! Bias: {:.6} rad/s", self.gyro_bias);
                }
            }
        } else {
            // Once calibrated, slowly track slow thermal drift when stationary (> 2 seconds)
            let time_since_encoder_activity = if timestamp_ms >= self.last_encoder_activity_ms {
                timestamp_ms - self.last_encoder_activity_ms
            } else {
                0
            };
            if time_since_encoder_activity > 2000 {
                let alpha_bias = 0.01;
                self.gyro_bias = alpha_bias * gyro_z + (1.0 - alpha_bias) * self.gyro_bias;
            }
        }
    }

    /// Update odometry with encoder updates and fuse with latest gyroscope reading.
    pub fn update_encoders(&mut self, left_delta: i32, right_delta: i32, timestamp_ms: u32) {
        if left_delta != 0 || right_delta != 0 {
            self.last_encoder_activity_ms = timestamp_ms;
            
            // If movement is detected during calibration, reset calibration samples
            if self.stationary_samples < 30 {
                self.stationary_samples = 0;
                self.stationary_gyro_sum = 0.0;
            }
        }

        self.cumulative_left += left_delta;
        self.cumulative_right += right_delta;

        let d_left_m = left_delta as f32 / self.ticks_per_meter;
        let d_right_m = right_delta as f32 / self.ticks_per_meter;

        let d_center = (d_left_m + d_right_m) / 2.0;
        let d_theta_encoders = (d_right_m - d_left_m) / self.wheel_base;

        let dt = match self.last_timestamp_ms {
            Some(last_ms) => {
                let diff = if timestamp_ms >= last_ms {
                    timestamp_ms - last_ms
                } else {
                    (u32::MAX - last_ms) + timestamp_ms + 1
                };
                (diff as f32 / 1000.0).min(1.0)
            }
            None => 0.1, // Fallback to 100ms
        };
        self.last_timestamp_ms = Some(timestamp_ms);

        // Compute theta delta using sensor fusion
        let d_theta = if self.has_imu && self.stationary_samples >= 30 {
            let d_theta_gyro = (self.last_gyro_z - self.gyro_bias) * dt;
            
            // Complementary filter: 98% Gyroscope, 2% Encoders.
            // Gyro handles short-term dynamic rotation without slip error,
            // while encoders anchor the heading to prevent long-term gyro drift/random walk.
            let alpha = 0.98;
            alpha * d_theta_gyro + (1.0 - alpha) * d_theta_encoders
        } else {
            // Fallback to pure encoder odometry if IMU is absent/uncalibrated
            d_theta_encoders
        };

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
        odom.update_encoders(1736, 1736, 100); // Approx 1 meter forward
        assert_eq!(odom.cumulative_left, 1736);
        assert_eq!(odom.cumulative_right, 1736);
        assert!((odom.pose.x - 1.0).abs() < 0.01);
        assert_eq!(odom.pose.y, 0.0);
        assert_eq!(odom.pose.theta, 0.0);

        // Turn in place
        odom.update_encoders(-425, 425, 200);
        assert_eq!(odom.cumulative_left, 1311);
        assert_eq!(odom.cumulative_right, 2161);
        assert!(odom.pose.theta > 0.0);
    }

    #[test]
    fn test_fused_odometry() {
        let mut odom = Odometry::new();
        
        // 1. Initially, no IMU, should fall back to encoder-only (turning left)
        odom.update_encoders(-10, 10, 100);
        assert!(odom.pose.theta > 0.0);
        
        // Reset
        odom = Odometry::new();
        
        // 2. Feed 30 IMU readings with 0.01 bias (stationary)
        for i in 1..=30 {
            odom.update_imu(0.01, i * 100);
        }
        assert_eq!(odom.stationary_samples, 30);
        assert!((odom.gyro_bias - 0.01).abs() < 0.0001);
        
        // 3. Now move with IMU.
        // With bias subtraction, if gyro reads 0.01, it should count as 0.0 angular velocity.
        // Let's do a straight line movement (left=100, right=100) and gyro reading = 0.01.
        odom.update_imu(0.01, 3100);
        odom.update_encoders(1736, 1736, 3100);
        
        // Heading should remain 0 because gyro delta is (0.01 - 0.01) * dt = 0
        // and encoder delta is 0.
        assert!((odom.pose.theta).abs() < 0.0001);
        
        // 4. Now perform a turn. Gyro reads 0.51 (which is 0.5 rad/s after bias subtraction).
        // Let's say dt = 0.1s (so angle change should be approx 0.05 rad from gyro).
        odom.update_imu(0.51, 3200);
        // Encoder delta is in-place turn: left = -10, right = 10.
        // Encoder d_theta = (10 - (-10)) / (1736.2 * 0.26) = 20 / 451.4 = 0.044305 rad
        // Fused delta = 0.98 * 0.05 + 0.02 * 0.044305 = 0.049886 rad.
        odom.update_encoders(-10, 10, 3200);
        
        assert!((odom.pose.theta - 0.049886).abs() < 0.001);
    }
}

