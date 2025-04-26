use byteorder::{NetworkEndian, ReadBytesExt};
use std::fmt;
use std::io::Cursor;
use crate::sensors::Sensor;

pub struct Imu {
    pub acceleration_x: f32,
    pub acceleration_y: f32,
    pub acceleration_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub magnetometer_x: f32,
    pub magnetometer_y: f32,
    pub magnetometer_z: f32,
}

impl Sensor for Imu {
    fn parse(data: &Vec<u8>) -> Self {
        let mut rdr = Cursor::new(data);

        // Read accelerometer data (3 float values)
        let acceleration_x = rdr.read_f32::<NetworkEndian>().unwrap();
        let acceleration_y = rdr.read_f32::<NetworkEndian>().unwrap();
        let acceleration_z = rdr.read_f32::<NetworkEndian>().unwrap();

        // Read gyroscope data (3 float values)
        let gyro_x = rdr.read_f32::<NetworkEndian>().unwrap();
        let gyro_y = rdr.read_f32::<NetworkEndian>().unwrap();
        let gyro_z = rdr.read_f32::<NetworkEndian>().unwrap();

        // Read magnetometer data (3 float values)
        let magnetometer_x = rdr.read_f32::<NetworkEndian>().unwrap();
        let magnetometer_y = rdr.read_f32::<NetworkEndian>().unwrap();
        let magnetometer_z = rdr.read_f32::<NetworkEndian>().unwrap();

        Imu {
            acceleration_x,
            acceleration_y,
            acceleration_z,
            gyro_x,
            gyro_y,
            gyro_z,
            magnetometer_x,
            magnetometer_y,
            magnetometer_z,
        }
    }
}

impl Imu {
    pub fn new() -> Self {
        Imu {
            acceleration_x: 0.0,
            acceleration_y: 0.0,
            acceleration_z: 0.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            magnetometer_x: 0.0,
            magnetometer_y: 0.0,
            magnetometer_z: 0.0,
        }
    }
}


impl fmt::Display for Imu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "IMU:\n  Acceleration (x,y,z): ({:.2}, {:.2}, {:.2})\n  Gyroscope (x,y,z): ({:.2}, {:.2}, {:.2})\n  Magnetometer (x,y,z): ({:.2}, {:.2}, {:.2})",
            self.acceleration_x, self.acceleration_y, self.acceleration_z,
            self.gyro_x, self.gyro_y, self.gyro_z,
            self.magnetometer_x, self.magnetometer_y, self.magnetometer_z
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_parse() {
        // Create test data with known values (9 * 4 bytes for f32 values)
        let test_data = vec![
            0x40, 0x00, 0x00, 0x00,  // acceleration_x = 2.0
            0x40, 0x40, 0x00, 0x00,  // acceleration_y = 3.0
            0x40, 0x80, 0x00, 0x00,  // acceleration_z = 4.0
            0x40, 0xA0, 0x00, 0x00,  // gyro_x = 5.0
            0x40, 0xC0, 0x00, 0x00,  // gyro_y = 6.0
            0x40, 0xE0, 0x00, 0x00,  // gyro_z = 7.0
            0x41, 0x00, 0x00, 0x00,  // magnetometer_x = 8.0
            0x41, 0x10, 0x00, 0x00,  // magnetometer_y = 9.0
            0x41, 0x20, 0x00, 0x00,  // magnetometer_z = 10.0
        ];

        let imu = Imu::parse(&test_data);

        assert_eq!(imu.acceleration_x, 2.0);
        assert_eq!(imu.acceleration_y, 3.0);
        assert_eq!(imu.acceleration_z, 4.0);
        assert_eq!(imu.gyro_x, 5.0);
        assert_eq!(imu.gyro_y, 6.0);
        assert_eq!(imu.gyro_z, 7.0);
        assert_eq!(imu.magnetometer_x, 8.0);
        assert_eq!(imu.magnetometer_y, 9.0);
        assert_eq!(imu.magnetometer_z, 10.0);
    }
}