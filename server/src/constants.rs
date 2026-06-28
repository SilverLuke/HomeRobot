pub const BUFFER_SIZE: usize = 16384;

use std::sync::Mutex;

#[derive(Debug, Clone, Copy)]
pub struct RobotSizes {
    pub wheel_base: f32,
    pub ticks_per_meter: f32,
}

pub static ROBOT_SIZES: Mutex<RobotSizes> = Mutex::new(RobotSizes {
    wheel_base: 0.26,
    ticks_per_meter: 1736.2,
});

