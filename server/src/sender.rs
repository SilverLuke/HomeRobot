use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use byteorder::{NetworkEndian, WriteBytesExt};
use crate::{SendPacketType};
use crate::reader::ProtocolManager;
use crate::SendPacketType::TxMotorMove;


// Motor command structure
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorCommand {
    pub(crate) right_power: u8,
    pub(crate) right_angle: f32,
    pub(crate) left_power: u8,
    pub(crate) left_angle: f32,
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self {
            right_power: 0,
            right_angle: 0.0,
            left_power: 0,
            left_angle: 0.0,
        }
    }
}


pub fn send_manual_command(motor_command: Arc<Mutex<MotorCommand>>, protocol: &mut ProtocolManager<TcpStream>, start_time: Instant, last_sent_command: &mut MotorCommand) {
    let mut new_command = None;
    if let Ok(current_command) = motor_command.lock() {
        new_command = Some(*current_command);
    }

    // Send motor commands periodically, but only if they've changed
    if let Some(current_command) = new_command {
        // Only send it if the command is different from the last one sent
        if current_command != *last_sent_command {
            let millis = start_time.elapsed().as_millis() as u32;
            let packet = craft_motor_packet(
                current_command.right_power,
                current_command.right_angle,
                current_command.left_power,
                current_command.left_angle,
                millis
            );

            println!("Sending motor command: Left power: {}, Right power: {}",
                     current_command.left_power as f32 * current_command.left_angle, 
                     current_command.right_power as f32 * current_command.right_angle);

            if let Err(e) = protocol.send_packet(&packet) {
                eprintln!("Error sending motor command: {:?}", e);
            }

            *last_sent_command = current_command;
        }
    }
}
fn craft_header(millis:u32, packet_type: SendPacketType, length: u16) -> Vec<u8> {
    let mut header = vec![0u8; 7];
    // In the firsts 4 bytes put millis
    header[0..4].copy_from_slice(&(millis.to_be_bytes()));

    // Then the type
    header[4] = packet_type as u8;

    // 2 bytes for the length
    header[5..7].copy_from_slice(&(length.to_be_bytes()));
    header
}
fn craft_motor_packet(right_power: u8, right_angle: f32, left_power:u8, left_angle: f32, millis:u32) -> Vec<u8> {
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
}