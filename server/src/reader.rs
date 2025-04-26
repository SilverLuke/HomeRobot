use crate::{craft_motor_packet, parse_battery_packet, parse_config_packet, parse_echo_packet, parse_encoder_packet, parse_imu_packet, HomeRobotPacket, PacketType, ReceivePacketType, BUFFER_SIZE, PREFIX_SIZE};
use byteorder::{NetworkEndian, ReadBytesExt};
use circular_buffer::CircularBuffer;
use std::io;
use std::io::{Read, Write};
use crate::sensors::{lidar, Sensor};
use crate::sensors::imu::Imu;
use crate::sensors::lidar::Lidar;

const HEADER_SIZE: usize = 7;
struct MessageHeader {
    sequence_millis:u32,
    packet_type: PacketType,
    size: u16,
}

pub struct ProtocolManager<S: Read + Write>{
    stream: S,
    read_buffer: Box<CircularBuffer<BUFFER_SIZE, u8>>,

    current_header: Option<MessageHeader>,
}

impl<S: Read + Write> ProtocolManager<S> {
    pub(crate) fn new(stream: S) -> ProtocolManager<S> {
        ProtocolManager {
            stream,
            read_buffer: CircularBuffer::<BUFFER_SIZE, u8>::boxed(),
            current_header: None,
        }
    }

    pub(crate) fn read_message(&mut self) -> Option<HomeRobotPacket> {
        match self.do_read() {
            Ok(0) => {},
            Ok(_) => {
                if self.current_header.is_none() {
                    self.parse_header();
                }
                if let Some(packet) = self.fetch_body() {
                    self.current_header = None;
                    return Some(packet);
                }
            }
            Err(e) => eprintln!("Error processing packet: {}", e),
        }
        None
    }

    fn parse_header(&mut self) {
        if self.read_buffer.len() < HEADER_SIZE {
            return;
        }

        // Buffer to read the packet's header (4 bytes for millis, 1 byte for type, 2 bytes for size)
        println!("Read first {PREFIX_SIZE} bytes");

        // Read millis (u32, 4 bytes)
        let millis = self.read_buffer.read_u32::<NetworkEndian>().unwrap();
        // let millis = self.read_buffer[0..4].read_u32::<LittleEndian>().unwrap();
        println!("Millis: {}", millis);

        // Read sensor type (u8, 1 byte)
        let packet_type_value_raw = self.read_buffer.read_u8().unwrap();
        let packet_type = ReceivePacketType::try_from(packet_type_value_raw).unwrap();
        println!("Packet type {packet_type:?}");

        // Read the size of the data (u16, 2 bytes)
        let size = self.read_buffer.read_u16::<NetworkEndian>().unwrap();
        println!("Data size: {size}");

        self.current_header = Some(MessageHeader {
            sequence_millis: millis,
            packet_type: PacketType::Receive(packet_type),
            size,
        })
    }

    fn fetch_body(&mut self) -> Option<HomeRobotPacket> {
        if let Some(header) = &self.current_header {
            if self.read_buffer.len() >= header.size as usize {
                let mut data = Box::new(Vec::with_capacity(header.size as usize));
                self.read_buffer.read_exact(&mut data).expect("This should not occur");
                return Some(HomeRobotPacket{
                    sequence_millis: header.sequence_millis,
                    packet_type: header.packet_type,
                    size: header.size,
                    data,
                });
            }
        }
        None
    }


    // Function to handle individual packets from the stream
    fn do_read(&mut self) -> io::Result<usize> {
        println!("Receive header");

        let mut buffer: Box<[u8]> = Box::new([0u8; BUFFER_SIZE / 2]);
        match self.stream.read(&mut buffer) {
            Ok(0) => {
                Ok(0)
            },
            Ok(n) => {
                self.read_buffer.extend(&buffer[..n]);
                Ok(n)
            }
            Err(error) => {
                Err(error)
            }
        }
    }


    pub(crate) fn send_message(&mut self, millis: u32) {
        let motor_packet =
            // Every second changes the robot motion
            if (millis / 5000) % 2 == 0 {
                // Go forward
                println!("Sending motor packet, forward. {millis}");
                craft_motor_packet(50, f32::INFINITY, 50, f32::INFINITY, millis)
            } else {
                // Go backward
                println!("Sending motor packet, backward. {millis}");
                craft_motor_packet(-50, f32::INFINITY, -50, f32::INFINITY, millis)
            };

        self.stream.write_all(&motor_packet).unwrap()
    }
    
    fn parse_data(packet: &HomeRobotPacket) {
        assert_eq!(packet.size as usize, packet.data.len(), "Vector data and size must be equal");
        // Extract fields directly from the byte vector
        match &packet.packet_type {
            PacketType::Send(_) => {}
            PacketType::Receive(rx_type) => {
                match rx_type {
                    ReceivePacketType::RxLidar => {
                        Lidar::parse(&packet.data);
                    }
                    ReceivePacketType::RxImu => {
                        Imu::parse(&packet.data);
                    }
                    ReceivePacketType::RxBattery => {
                        parse_battery_packet(packet.size, &packet.data);
                    }
                    ReceivePacketType::RxEncoderMotor => {
                        parse_encoder_packet(packet.size, &packet.data);
                    }
                    ReceivePacketType::RxConfig => {
                        parse_config_packet(packet.size, &packet.data);
                    }
                    ReceivePacketType::RxEcho => {
                        parse_echo_packet(packet.size, &packet.data);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::io::{self, Read, Write};
    use mockall::{mock, predicate::*};
    use super::ProtocolManager;

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

    #[test]
    fn send_writes_all_bytes() -> io::Result<()> {
        let mut mock = MockStream::new();
        mock.expect_write()
            .withf(|buf: &[u8]| {
                // Verify the packet structure
                buf.len() == 17  // 7 bytes header + 10 bytes motor data
            })
            .returning(|buf| Ok(buf.len()));

        let mut proto = ProtocolManager::new(mock);
        proto.send_message(1000);
        Ok(())
    }

    #[test]
    fn test_read_message() -> io::Result<()> {
        let mut mock = MockStream::new();
        
        // Setup mock to return a valid packet
        mock.expect_read()
            .returning(|buf| {
                // Simulate a packet with header and some data
                let packet = [
                    0, 0, 0, 100,  // millis (u32)
                    1,             // packet type
                    0, 4,         // size (u16)
                    1, 2, 3, 4    // data
                ];
                let len = std::cmp::min(buf.len(), packet.len());
                buf[..len].copy_from_slice(&packet[..len]);
                Ok(len)
            });

        let mut proto = ProtocolManager::new(mock);
        let message = proto.read_message();
        assert!(message.is_some());
        Ok(())
    }
}