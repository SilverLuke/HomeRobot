pub mod lidar;
pub mod imu;

pub trait Sensor {
    fn parse(data: &Vec<u8>) -> Self;
}

