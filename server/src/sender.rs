use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use byteorder::{NetworkEndian, WriteBytesExt};
use crate::{SendPacketType};
use crate::reader::ProtocolManager;
use crate::SendPacketType::TxMotorMove;


// Motor command structure
#[derive(Debug, Clone, PartialEq)]
pub enum MotorCommand {
    Stop,
    Direct { 
        left_speed: i16,
        right_speed: i16,
    }, // Motor speed directly from -255 to 255
    Angle { 
        left_power: u8, 
        left_angle: f32,
        right_power: u8,
        right_angle: f32,
    }, // Existing angle-based movement
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self::Angle {
            left_power: 0,
            left_angle: 0.0,
            right_power: 0,
            right_angle: 0.0,
        }
    }
}


pub fn send_manual_command(
    motor_command: Arc<Mutex<MotorCommand>>,
    protocol: &mut ProtocolManager<TcpStream>,
    start_time: Instant,
    last_sent_command: &mut MotorCommand)
{
    let mut new_command = None;
    if let Ok(current_command) = motor_command.lock() {
        new_command = Some(current_command.clone());
    }

    // Send motor commands periodically, but only if they've changed
    if let Some(current_command) = new_command {
        // Only send it if the command is different from the last one sent
        if current_command != *last_sent_command {
            let millis = start_time.elapsed().as_millis() as u32;

            let packet = serialize_command(&current_command, millis);

            println!(
                "Sending motor command: {:?}, {:?}",
                current_command,
                packet.iter().map(|b| format!("{:02x} ", b)).collect::<String>()
            );

            if let Err(e) = protocol.send_packet(&packet) {
                eprintln!("Error sending motor command: {:?}", e);
            }

            *last_sent_command = current_command;
        }
    }
}

fn serialize_command(current_command: &MotorCommand, millis: u32) -> Vec<u8> {
    let packet = match current_command {
        MotorCommand::Stop => {
            craft_motor_packet(0, 0.0, 0, 0.0, millis)
        }
        MotorCommand::Direct { right_speed, left_speed } => {
            // Normalize direct command speeds to power and angles
            let left_power = left_speed.abs() as u8;
            let left_angle = if *left_speed >= 0 { 1.0 } else { -1.0 };
            let right_power = right_speed.abs() as u8;
            let right_angle = if *right_speed >= 0 { 1.0 } else { -1.0 };
            craft_motor_packet(left_power, left_angle, right_power, right_angle, millis)
        }
        MotorCommand::Angle {
            right_power,
            right_angle,
            left_power,
            left_angle,
        } => {
            craft_motor_packet(*left_power, *left_angle, *right_power, *right_angle, millis)
        }
    };
    packet
}

fn craft_header(millis:u32, packet_type: SendPacketType, length: u16) -> Vec<u8> {
    let mut header = vec![0u8; 7];
    // In the firsts 4 bytes put millis
    (&mut header[0..4]).write_u32::<NetworkEndian>(millis).unwrap();

    // Then the type
    header[4] = packet_type as u8;

    // 2 bytes for the length
    (&mut header[5..7]).write_u16::<NetworkEndian>(length).unwrap();
    header
}
fn craft_motor_packet(left_power:u8, left_angle: f32, right_power: u8, right_angle: f32, millis:u32) -> Vec<u8> {
    let header = craft_header(millis, TxMotorMove, 10);

    // 17 is the length of the entire packet header 7 + 10 of the body
    let mut motor_packet = vec![0u8; 10];

    // Actual data
    // Right motor
    motor_packet[0] = right_power;
    (&mut motor_packet[1..5]).write_f32::<NetworkEndian>(right_angle).unwrap();
    // Left motor
    motor_packet[5] = left_power;
    (&mut motor_packet[6..10]).write_f32::<NetworkEndian>(left_angle).unwrap();

    [header, motor_packet].concat()
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::SendPacketType::TxMotorMove;
    use super::ProtocolManager;
    use crate::{PacketType, ReceivePacketType};
    use mockall::mock;
    use std::io::{self, Read, Write};

    mock! {
        pub Stream {
        }
        
        impl Read for Stream {
            fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
        }
        
        impl Write for Stream {
            fn write(&mut self, buf: &[u8]) -> io::Result<usize>;
            fn flush(&mut self) -> io::Result<()>;
        }
    }

    // #[test]
    // fn send_writes_all_bytes() -> io::Result<()> {
    //     let mut mock = MockStream::new();
    //     mock.expect_write()
    //         .withf(|buf: &[u8]| {
    //             // Verify the packet structure
    //             buf.len() == 17  // 7 bytes header + 10 bytes motor data
    //         })
    //         .returning(|buf| Ok(buf.len()));
    // 
    //     let mut proto = ProtocolManager::new(mock);
    //     proto.send_message(1000);
    //     Ok(())
    // }

    #[test]
    fn test_stop_command_serialization() {
        // Create test inputs
        let millis = 255; // 00 00 4d fc in hex

        // Call the function to craft the packet
        let packet = serialize_command(
            &MotorCommand::Stop,
            millis,
        );

        // Expected packet
        let expected_packet: Vec<u8> = vec![
            0x00, 0x00, 0x00, 0xff, // Millis
            0x00,                   // SendPacketType::TxMotorMove
            0x00, 0x0a,             // Length = 10
            0x00,                   // Right power = 0
            0x00, 0x00, 0x00, 0x00, // Right angle = 0.0
            0x00,                   // Left power = 0
            0x00, 0x00, 0x00, 0x00, // Left angle = 0.0
        ];

        // Assert that the created packet matches the expected packet
        assert_eq!(packet, expected_packet, "Generated packet does not match expected format");
    }


    #[test]
    fn test_direct_command_serialization() {
        // Create test inputs
        let millis = 255; // 00 00 4d fc in hex

        // Call the function to craft the packet
        let packet = serialize_command(
            &MotorCommand::Direct { left_speed: 128, right_speed: 0 },
            millis,
        );

        // Expected packet
        let expected_packet: Vec<u8> = vec![
            0x00, 0x00, 0x00, 0xff, // Millis
            0x00,                   // SendPacketType::TxMotorMove
            0x00, 0x0a,             // Length = 10
            0x00,                   // Right power = 0
            0x00, 0x00, 0x00, 0x00, // Right angle = 0.0
            0x00,                   // Left power = 0
            0x00, 0x00, 0x00, 0x00, // Left angle = 0.0
        ];

        // Assert that the created packet matches the expected packet
        assert_eq!(packet, expected_packet, "Generated packet does not match expected format");
    }

}