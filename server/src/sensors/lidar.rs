use std::f32::consts::PI;
use byteorder::{NetworkEndian, ReadBytesExt};
pub(crate) use crate::sensors::Sensor;
use std::fmt;
use std::io::Cursor;

#[derive(Debug)]
struct Point {
    /**
    This struct indicates a point in the space relative to the robot.
    The standard unit is millimeters.
    */
    x: Option<f32>,
    y: Option<f32>,
    distance: f32,
    angle: f32,
    quality: u8,
    last: bool
}

impl Point {
    fn new(distance: f32, angle:f32, quality:u8, last:bool) -> Self {
        Self {
            x: None,
            y: None,
            distance,
            angle,
            quality,
            last
        }
    }
    
    fn vector(&mut self) -> (f32, f32) {
        // Calculate position: x, y based on angle and distance
        if self.x.is_none() || self.y.is_none() {
            let angle_rad = self.angle * PI / 180.0;
            let distance_meters = self.distance;
            self.x = Some(distance_meters * f32::cos(angle_rad));
            self.y = Some(distance_meters * f32::sin(angle_rad));
        }
        (self.x.unwrap(), self.y.unwrap())
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Packet Type: Lidar, Completed: {}, Distance (mm): {:.2}, Angle (deg): {:.2}, Quality: {}",
            self.last,
            self.distance,
            self.angle,
            self.quality
        )
    }
}

const SINGLE_PACKET_SIZE: usize = 5;
pub(crate) struct Lidar {
    points: Vec<Point>
}

impl Sensor for Lidar {
    fn parse(data: &Vec<u8>) -> Self {
        // Size of a single lidar.rs packet in bytes
        assert_eq!(data.len() % SINGLE_PACKET_SIZE, 0, "Check if all packets are complete");

        let packets = data.len() / SINGLE_PACKET_SIZE;

        let mut rdr = Cursor::new(data);
        let mut points = Vec::new();
        
        for i in 0..packets {
            let point = parse_single(&mut rdr);
            points.push(point);
        }

        fn parse_single(rdr: &mut Cursor<&Vec<u8>>) -> Point {
            let sync_quality = rdr.read_u8().unwrap();
            let angle_q6_check_bit = rdr.read_u16::<NetworkEndian>().unwrap();
            let distance_q2 = rdr.read_u16::<NetworkEndian>().unwrap();

            // Perform the operations
            let scan_completed = (sync_quality & (0x1<<0)) != 0; // Extract syncbit
            let distance_mm = (distance_q2 as f32) * 0.25;
            let angle_deg = ((angle_q6_check_bit >> 1) as f32) * 0.015625; // Shift and scale
            let quality = (sync_quality >> 2) & 0x3F; // Extract the 6-bit quality

            Point::new(distance_mm, angle_deg, quality, scan_completed)
        }
        
        Self {points}
    }
    

}

impl fmt::Display for Lidar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for point in &self.points {
            writeln!(f, "{}", point)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_point_new() {
        let point = Point::new(100.0, 45.0, 63, false);
        assert_eq!(point.distance, 100.0);
        assert_eq!(point.angle, 45.0);
        assert_eq!(point.quality, 63);
        assert_eq!(point.last, false);
        assert!(point.x.is_none());
        assert!(point.y.is_none());
    }

    #[test]
    fn test_point_vector() {
        let mut point = Point::new(100.0, 45.0, 63, false);
        let (x, y) = point.vector();

        // For 45 degrees, x and y should be approximately equal
        // Using a small epsilon for float comparison
        let expected_coord = 100.0 * f32::cos(PI / 4.0);
        assert!((x - expected_coord).abs() < 0.001);
        assert!((y - expected_coord).abs() < 0.001);

        // Test caching - coordinates should remain the same
        let (x2, y2) = point.vector();
        assert_eq!(x, x2);
        assert_eq!(y, y2);
    }

    #[test]
    fn test_lidar_parse() {
        // Create a test packet:
        // - sync_quality: 0b00000101 (quality=1, completed=1)
        // - angle: 180 degrees (12000 in Q6 format)
        // - distance: 1000mm (4000 in Q2 format)
        let test_data = vec![
            0x05, // sync_quality
            0x5A, 0x00, // angle 180
            0x0F, 0xA0  // distance 1000
        ];

        let lidar = Lidar::parse(&test_data);
        assert_eq!(lidar.points.len(), 1);

        let point = &lidar.points[0];
        assert_eq!(point.quality, 1);
        assert_eq!(point.last, true);
        assert_eq!(point.angle, 180.);
        assert_eq!(point.distance, 1000.);
    }

    #[test]
    #[should_panic(expected = "Check if all packets are complete")]
    fn test_lidar_parse_incomplete_packet() {
        let incomplete_data = vec![0x05, 0x2E, 0xE0, 0x0F]; // Only 4 bytes instead of 5
        Lidar::parse(&incomplete_data);
    }

    #[test]
    fn test_lidar_parse_multiple_packets() {
        // Two identical packets
        let test_data = vec![
            0x05, 0x2E, 0xE0, 0x0F, 0xA0,
            0x05, 0x2E, 0xE0, 0x0F, 0xA0
        ];

        let lidar = Lidar::parse(&test_data);
        assert_eq!(lidar.points.len(), 2);
    }

    #[test]
    fn test_point_display() {
        let point = Point::new(100.0, 45.0, 63, true);
        let display_string = format!("{}", point);
        assert!(display_string.contains("Distance (mm): 100.00"));
        assert!(display_string.contains("Angle (deg): 45.00"));
        assert!(display_string.contains("Quality: 63"));
        assert!(display_string.contains("Completed: true"));
    }
}